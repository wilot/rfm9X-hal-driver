// For embedded-hal 1.0+
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

// Error type without heap allocation
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RFMError {
    InvalidVersion { version: u8 },
    ModeChangeFailed { old: u8, new: u8, set: u8 },
    TransmissionTimedOut,
    Spi,
    Gpio,
    InvalidPacketSize,
}

// Register definitions
#[allow(dead_code)]
#[derive(Copy, Clone)]
#[repr(u8)]
enum Register {
    FIFO = 0x00,
    OpMode = 0x01,
    FSKBitrateMSB = 0x02,
    FSKBitrateLSB = 0x03,
    FSKFdevMSB = 0x04,
    FSKFdevLSB = 0x05,
    FRFMSB = 0x06,
    FRFMID = 0x07,
    FRFLSB = 0x08,
    PAConfig = 0x09,
    FIFOAddressPointer = 0x0D,
    FIFOTXBaseAddress = 0x0E,
    FIFORXBaseAddress = 0x0F,
    IRQFlags = 0x12,
    PayloadLength = 0x22,
    ModemConfig1 = 0x1D,
    ModemConfig2 = 0x1E,
    ModemConfig3 = 0x26,
    FSKSyncConfig = 0x27,
    FSKSyncValue1 = 0x28,
    PreambleLengthMSB = 0x20,
    PreambleLengthLSB = 0x21,
    SymbolTimeoutLSB = 0x1F,
    FSKPacketConfig1 = 0x30,
    FSKPacketConfig2 = 0x31,
    FSKPayloadLength = 0x32,
    FSKFifoThreshold = 0x35,
    Timer1Coefficient = 0x39,
    FSKIRQFlags1 = 0x3E,
    FSKIRQFlags2 = 0x3F,
    DIOMapping1 = 0x40,
    Version = 0x42,
}

// Mode flags using const
pub mod mode {
    pub const SLEEP: u8 = 0b000;
    pub const STANDBY: u8 = 0b001;
    pub const TRANSMIT: u8 = 0b011;
    pub const RECEIVE_CONTINUOUS: u8 = 0b101;
    pub const LORA: u8 = 0b1000_0000;
}

pub mod modem_config {
    pub const BW125: u8 = 0b0111_0000;
    pub const CODING_RATE_4_5: u8 = 0b0000_0010;
    pub const SF7: u8 = 0x70;
    pub const SF8: u8 = 0x80;
    pub const SF9: u8 = 0x90;
    pub const AUTO_AGC_ON: u8 = 0b0000_0100;
    pub const RX_PAYLOAD_CRC: u8 = 0b0000_0100;
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DataRate {
    SF7BW125,
    SF8BW125,
    SF9BW125,
}

impl DataRate {
    fn modem_config_1(&self) -> u8 {
        modem_config::BW125 | modem_config::CODING_RATE_4_5
    }

    fn modem_config_2(&self) -> u8 {
        match self {
            DataRate::SF7BW125 => modem_config::SF7,
            DataRate::SF8BW125 => modem_config::SF8,
            DataRate::SF9BW125 => modem_config::SF9,
        }
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum FskDataRate {
        BR50kFd25k,
        BR100kFd50k,
        BR150kFd75k,
    }

    impl FskDataRate {
        fn bitrate_registers(&self) -> [u8; 2] {
            // FXOSC is 32 MHz, bitrate register = FXOSC / bitrate
            match self {
                FskDataRate::BR50kFd25k => [0x02, 0x80],  // 640
                FskDataRate::BR100kFd50k => [0x01, 0x40], // 320
                FskDataRate::BR150kFd75k => [0x00, 0xD5], // 213
            }
        }

        fn fdev_registers(&self) -> [u8; 2] {
            // FSTEP is 61.03515625 Hz, fdev register = fdev / FSTEP
            match self {
                FskDataRate::BR50kFd25k => [0x01, 0x9A],  // ~25 kHz
                FskDataRate::BR100kFd50k => [0x03, 0x33], // ~50 kHz
                FskDataRate::BR150kFd75k => [0x04, 0xCD], // ~75 kHz
            }
        }
    }

    fn modem_config_3(&self) -> u8 {
        modem_config::AUTO_AGC_ON
    }
}

// EU868 frequencies (can add more bands)
pub const FREQ_CH0: [u8; 3] = [0xD9, 0x06, 0x8B]; // 868.100 MHz
pub const FREQ_CH1: [u8; 3] = [0xD9, 0x13, 0x58]; // 868.300 MHz
pub const FREQ_CH2: [u8; 3] = [0xD9, 0x20, 0x24]; // 868.500 MHz

// Main driver struct with generic HAL traits
pub struct RFM95<SPI, RST, DIO0, DELAY>
where
    SPI: SpiDevice,
    RST: OutputPin,
    DIO0: InputPin,
    DELAY: DelayNs,
{
    spi: SPI,
    reset: RST,
    dio0: DIO0,
    delay: DELAY,
}

impl<SPI, RST, DIO0, DELAY> RFM95<SPI, RST, DIO0, DELAY>
where
    SPI: SpiDevice,
    RST: OutputPin,
    DIO0: InputPin,
    DELAY: DelayNs,
{
    /// Create a new RFM95 driver instance
    ///
    /// Note: SpiDevice in embedded-hal 1.0 handles CS internally
    pub fn new(spi: SPI, reset: RST, dio0: DIO0, delay: DELAY) -> Self {
        RFM95 {
            spi,
            reset,
            dio0,
            delay,
        }
    }

    /// Reset and initialize the RFM95 module
    pub fn reset(&mut self) -> Result<(), RFMError> {
        // Pull reset low
        self.reset.set_low().map_err(|_| RFMError::Gpio)?;
        self.delay.delay_ms(10);

        // Release reset
        self.reset.set_high().map_err(|_| RFMError::Gpio)?;
        self.delay.delay_ms(10);

        // Check version
        let version = self.read_register(Register::Version)?;
        if version != 0x12 {
            return Err(RFMError::InvalidVersion { version });
        }

        // Initialize chip
        self.set_mode(mode::SLEEP)?;
        self.set_mode(mode::SLEEP | mode::LORA)?;

        // Configure power (max)
        self.write_register(Register::PAConfig, 0xFF)?;

        // Timeouts and preamble
        self.write_register(Register::SymbolTimeoutLSB, 0x25)?;
        self.write_register(Register::PreambleLengthMSB, 0x00)?;
        self.write_register(Register::PreambleLengthLSB, 0x08)?;

        // LoRa sync word
        self.write_register(Register::Timer1Coefficient, 0x34)?;

        // FIFO pointers
        self.write_register(Register::FIFOTXBaseAddress, 0x80)?;
        self.write_register(Register::FIFORXBaseAddress, 0x00)?;

        Ok(())
    }

    fn set_mode(&mut self, mode: u8) -> Result<(), RFMError> {
        let old_mode = self.read_register(Register::OpMode)?;

        if old_mode == mode {
            return Ok(());
        }

        self.write_register(Register::OpMode, mode)?;
        self.delay.delay_ms(10);

        let set_mode = self.read_register(Register::OpMode)?;
        if set_mode != mode {
            return Err(RFMError::ModeChangeFailed {
                old: old_mode,
                new: mode,
                set: set_mode,
            });
        }
        Ok(())
    }

    fn read_register(&mut self, register: Register) -> Result<u8, RFMError> {
        let cmd = (register as u8) & 0x7F; // Read command
        let mut buffer = [cmd, 0u8];

        // In embedded-hal 1.0, SpiDevice handles CS automatically
        self.spi
            .transfer_in_place(&mut buffer)
            .map_err(|_| RFMError::Spi)?;

        Ok(buffer[1])
    }

    fn write_register(&mut self, register: Register, value: u8) -> Result<(), RFMError> {
        let cmd = (register as u8) | 0x80; // Write command
        let buffer = [cmd, value];

        self.spi.write(&buffer).map_err(|_| RFMError::Spi)?;

        Ok(())
    }

    fn set_frequency(&mut self, freq: [u8; 3]) -> Result<(), RFMError> {
        self.write_register(Register::FRFMSB, freq[0])?;
        self.write_register(Register::FRFMID, freq[1])?;
        self.write_register(Register::FRFLSB, freq[2])?;
        Ok(())
    }

    fn set_data_rate(&mut self, data_rate: DataRate, enable_crc: bool) -> Result<(), RFMError> {
        let mut modem_config_2 = data_rate.modem_config_2();
        if enable_crc {
            modem_config_2 |= modem_config::RX_PAYLOAD_CRC;
        }

        self.write_register(Register::ModemConfig1, data_rate.modem_config_1())?;
        self.write_register(Register::ModemConfig2, modem_config_2)?;
        self.write_register(Register::ModemConfig3, data_rate.modem_config_3())?;
        Ok(())
    }

    fn set_fsk_data_rate(&mut self, data_rate: FskDataRate) -> Result<(), RFMError> {
        let bitrate = data_rate.bitrate_registers();
        let fdev = data_rate.fdev_registers();

        self.write_register(Register::FSKBitrateMSB, bitrate[0])?;
        self.write_register(Register::FSKBitrateLSB, bitrate[1])?;
        self.write_register(Register::FSKFdevMSB, fdev[0])?;
        self.write_register(Register::FSKFdevLSB, fdev[1])?;

        Ok(())
    }

    fn configure_fsk_packet_engine(&mut self) -> Result<(), RFMError> {
        // Variable packet length, CRC on.
        self.write_register(Register::FSKPacketConfig1, 0b1001_0000)?;
        self.write_register(Register::FSKPacketConfig2, 0x00)?;
        self.write_register(Register::FSKPayloadLength, 0xFF)?;
        // Tx starts when FIFO level threshold reached, threshold set to 15 bytes.
        self.write_register(Register::FSKFifoThreshold, 0x8F)?;
        // Enable sync word, one sync byte.
        self.write_register(Register::FSKSyncConfig, 0x88)?;
        // LoRa sync word 0x34 reused for interoperability in dual-mode setups.
        self.write_register(Register::FSKSyncValue1, 0x34)?;
        Ok(())
    }

    /// Send a packet with specified frequency and data rate
    pub fn send_packet(
        &mut self,
        packet: &[u8],
        frequency: [u8; 3],
        data_rate: DataRate,
    ) -> Result<(), RFMError> {
        if packet.is_empty() || packet.len() > 255 {
            return Err(RFMError::InvalidPacketSize);
        }

        self.set_mode(mode::LORA | mode::STANDBY)?;

        // Configure DIO0 for TxDone
        self.write_register(Register::DIOMapping1, 0x40)?;

        self.set_frequency(frequency)?;
        self.set_data_rate(data_rate, true)?;
        self.write_register(Register::PayloadLength, packet.len() as u8)?;
        self.write_register(Register::FIFOAddressPointer, 0x80)?;

        // Write payload
        for &byte in packet {
            self.write_register(Register::FIFO, byte)?;
        }

        // Clear IRQ flags
        self.write_register(Register::IRQFlags, 0xFF)?;

        // Transmit
        self.set_mode(mode::LORA | mode::TRANSMIT)?;

        // Poll for TxDone (with timeout)
        for _ in 0..100 {
            self.delay.delay_ms(10);
            let irq_flags = self.read_register(Register::IRQFlags)?;
            if (irq_flags & 0b0000_1000) != 0 {
                // TxDone flag set
                self.write_register(Register::IRQFlags, 0xFF)?;
                self.set_mode(mode::LORA | mode::STANDBY)?;
                return Ok(());
            }
        }

        Err(RFMError::TransmissionTimedOut)
    }

    /// Enter continuous receive mode
    pub fn enter_receive_mode(
        &mut self,
        frequency: [u8; 3],
        data_rate: DataRate,
    ) -> Result<(), RFMError> {
        self.set_mode(mode::LORA | mode::STANDBY)?;
        self.set_frequency(frequency)?;
        self.set_data_rate(data_rate, true)?;

        // Configure DIO0 for RxDone
        self.write_register(Register::DIOMapping1, 0x00)?;

        self.set_mode(mode::LORA | mode::RECEIVE_CONTINUOUS)?;
        Ok(())
    }

    /// Send a packet in FSK mode using variable length packets.
    pub fn send_fsk_packet(
        &mut self,
        packet: &[u8],
        frequency: [u8; 3],
        data_rate: FskDataRate,
    ) -> Result<(), RFMError> {
        if packet.is_empty() || packet.len() > 255 {
            return Err(RFMError::InvalidPacketSize);
        }

        self.set_mode(mode::STANDBY)?;
        self.set_frequency(frequency)?;
        self.set_fsk_data_rate(data_rate)?;
        self.configure_fsk_packet_engine()?;

        // DIO0 = PacketSent for FSK TX.
        self.write_register(Register::DIOMapping1, 0x40)?;

        // Variable packet format: first byte is payload length.
        self.write_register(Register::FIFO, packet.len() as u8)?;
        for &byte in packet {
            self.write_register(Register::FIFO, byte)?;
        }

        self.set_mode(mode::TRANSMIT)?;

        // Wait for PacketSent bit in RegIrqFlags2.
        for _ in 0..100 {
            self.delay.delay_ms(10);
            let irq_flags_2 = self.read_register(Register::FSKIRQFlags2)?;
            if (irq_flags_2 & 0b0000_1000) != 0 {
                self.set_mode(mode::STANDBY)?;
                return Ok(());
            }
        }

        Err(RFMError::TransmissionTimedOut)
    }

    /// Enter continuous FSK receive mode using variable length packets.
    pub fn enter_fsk_receive_mode(
        &mut self,
        frequency: [u8; 3],
        data_rate: FskDataRate,
    ) -> Result<(), RFMError> {
        self.set_mode(mode::STANDBY)?;
        self.set_frequency(frequency)?;
        self.set_fsk_data_rate(data_rate)?;
        self.configure_fsk_packet_engine()?;

        // DIO0 = PayloadReady for FSK RX.
        self.write_register(Register::DIOMapping1, 0x00)?;

        self.set_mode(mode::RECEIVE_CONTINUOUS)?;
        Ok(())
    }

    /// Return true if an FSK packet is ready in the FIFO.
    pub fn is_fsk_packet_ready(&mut self) -> Result<bool, RFMError> {
        let irq_flags_2 = self.read_register(Register::FSKIRQFlags2)?;
        Ok((irq_flags_2 & 0b0000_0100) != 0)
    }

    /// Read one FSK packet from FIFO into `buffer`.
    pub fn read_fsk_packet(&mut self, buffer: &mut [u8]) -> Result<usize, RFMError> {
        let payload_length = self.read_register(Register::FIFO)? as usize;
        if payload_length == 0 || payload_length > buffer.len() {
            return Err(RFMError::InvalidPacketSize);
        }

        for byte in buffer.iter_mut().take(payload_length) {
            *byte = self.read_register(Register::FIFO)?;
        }
        Ok(payload_length)
    }

    /// Check if DIO0 pin is high (indicates RxDone or TxDone)
    pub fn is_dio0_high(&mut self) -> Result<bool, RFMError> {
        self.dio0.is_high().map_err(|_| RFMError::Gpio)
    }

    /// Return raw FSK IRQ flags for diagnostics.
    pub fn read_fsk_irq_flags(&mut self) -> Result<(u8, u8), RFMError> {
        Ok((
            self.read_register(Register::FSKIRQFlags1)?,
            self.read_register(Register::FSKIRQFlags2)?,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock types for testing
    struct MockSpi {
        registers: [u8; 256],
    }

    impl MockSpi {
        fn new() -> Self {
            let mut registers = [0u8; 256];
            registers[Register::Version as usize] = 0x12; // Set correct version
            MockSpi { registers }
        }
    }

    impl embedded_hal::spi::ErrorType for MockSpi {
        type Error = core::convert::Infallible;
    }

    impl SpiDevice for MockSpi {
        fn transaction(
            &mut self,
            operations: &mut [embedded_hal::spi::Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    embedded_hal::spi::Operation::Write(data) => {
                        if data.len() >= 2 {
                            let reg = data[0] & 0x7F;
                            self.registers[reg as usize] = data[1];
                        }
                    }
                    embedded_hal::spi::Operation::Transfer(read, write) => {
                        if write.len() >= 1 {
                            let reg = write[0] & 0x7F;
                            if read.len() >= 2 {
                                read[1] = self.registers[reg as usize];
                            }
                        }
                    }
                    embedded_hal::spi::Operation::TransferInPlace(data) => {
                        if data.len() >= 2 {
                            let reg = data[0] & 0x7F;
                            data[1] = self.registers[reg as usize];
                        }
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    struct MockPin(bool);

    impl embedded_hal::digital::ErrorType for MockPin {
        type Error = core::convert::Infallible;
    }

    impl OutputPin for MockPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0 = false;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0 = true;
            Ok(())
        }
    }

    impl InputPin for MockPin {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(self.0)
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.0)
        }
    }

    struct MockDelay;

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn test_data_rate_configs() {
        assert_eq!(DataRate::SF7BW125.modem_config_2(), modem_config::SF7);
        assert_eq!(DataRate::SF8BW125.modem_config_2(), modem_config::SF8);
        assert_eq!(DataRate::SF9BW125.modem_config_2(), modem_config::SF9);
    }

    #[test]
    fn test_reset_checks_version() {
        let spi = MockSpi::new();
        let reset = MockPin(true);
        let dio0 = MockPin(false);
        let delay = MockDelay;

        let mut rfm = RFM95::new(spi, reset, dio0, delay);
        assert!(rfm.reset().is_ok());
    }

    #[test]
    fn test_invalid_packet_size() {
        let spi = MockSpi::new();
        let reset = MockPin(true);
        let dio0 = MockPin(false);
        let delay = MockDelay;

        let mut rfm = RFM95::new(spi, reset, dio0, delay);
        rfm.reset().unwrap();

        let empty_packet = [];
        let result = rfm.send_packet(&empty_packet, FREQ_CH0, DataRate::SF7BW125);
        assert_eq!(result, Err(RFMError::InvalidPacketSize));
    }
}
