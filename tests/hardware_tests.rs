// tests/hardware_tests.rs
// Integration tests that run on actual Raspberry Pi hardware
//
// Run with: cargo test --test hardware_tests -- --ignored --nocapture --test-threads=1
// Note: Must be run on a Raspberry Pi with RFM98 module connected

use lora_rfm98::{DataRate, RFMError, FREQ_CH0, RFM95};
use rppal::gpio::Gpio;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use std::thread;
use std::time::Duration;

// Pin definitions for your setup
const SPI_BUS: Bus = Bus::Spi0;
const SPI_CS: SlaveSelect = SlaveSelect::Ss0; // GPIO7/CE0
const PIN_RESET: u8 = 12; // GPIO12 (DIO5 used as reset)
const PIN_DIO0: u8 = 16; // GPIO16
const PIN_LED: u8 = 21; // GPIO21 (optional status LED)

// === Wrapper types to bridge rppal to embedded-hal 1.0 ===

// Custom error type that implements embedded-hal 1.0 Error traits
#[derive(Debug)]
struct HalError;

impl embedded_hal::spi::Error for HalError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

impl embedded_hal::digital::Error for HalError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

// SPI wrapper
struct SpiWrapper {
    spi: Spi,
}

impl embedded_hal::spi::ErrorType for SpiWrapper {
    type Error = HalError;
}

impl embedded_hal::spi::SpiDevice for SpiWrapper {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                embedded_hal::spi::Operation::Read(buf) => {
                    self.spi.read(buf).map_err(|_| HalError)?;
                }
                embedded_hal::spi::Operation::Write(data) => {
                    self.spi.write(data).map_err(|_| HalError)?;
                }
                embedded_hal::spi::Operation::Transfer(read, write) => {
                    self.spi.transfer(read, write).map_err(|_| HalError)?;
                }
                embedded_hal::spi::Operation::TransferInPlace(buf) => {
                    // rppal doesn't have transfer_in_place, emulate it
                    let mut read_buf = vec![0u8; buf.len()];
                    self.spi
                        .transfer(&mut read_buf, buf)
                        .map_err(|_| HalError)?;
                    buf.copy_from_slice(&read_buf);
                }
                embedded_hal::spi::Operation::DelayNs(_) => {
                    // No-op
                }
            }
        }
        Ok(())
    }
}

// OutputPin wrapper
struct OutputPinWrapper {
    pin: rppal::gpio::OutputPin,
}

impl embedded_hal::digital::ErrorType for OutputPinWrapper {
    type Error = HalError;
}

impl embedded_hal::digital::OutputPin for OutputPinWrapper {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }
}

// InputPin wrapper
struct InputPinWrapper {
    pin: rppal::gpio::InputPin,
}

impl embedded_hal::digital::ErrorType for InputPinWrapper {
    type Error = HalError;
}

impl embedded_hal::digital::InputPin for InputPinWrapper {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_low())
    }
}

// Delay implementation
struct StdDelay;

impl embedded_hal::delay::DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        if ns < 1_000_000 {
            // For short delays, use spin loop
            let start = std::time::Instant::now();
            while start.elapsed().as_nanos() < ns as u128 {}
        } else {
            thread::sleep(Duration::from_nanos(ns as u64));
        }
    }
}

// === Helper function to create RFM95 instance ===

fn create_rfm95() -> Result<
    RFM95<SpiWrapper, OutputPinWrapper, InputPinWrapper, StdDelay>,
    Box<dyn std::error::Error>,
> {
    // Initialize SPI
    let spi = Spi::new(
        SPI_BUS,
        SPI_CS,
        10_000_000, // 10 MHz clock speed
        Mode::Mode0,
    )?;

    let spi_wrapper = SpiWrapper { spi };

    // Initialize GPIO pins
    let gpio = Gpio::new()?;

    let reset_pin = OutputPinWrapper {
        pin: gpio.get(PIN_RESET)?.into_output(),
    };

    let dio0_pin = InputPinWrapper {
        pin: gpio.get(PIN_DIO0)?.into_input(),
    };

    let delay = StdDelay;

    Ok(RFM95::new(spi_wrapper, reset_pin, dio0_pin, delay))
}

// === TESTS ===

#[test]
#[ignore]
fn test_rfm95_hardware_init() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    let result = rfm.reset();
    assert!(result.is_ok(), "Reset failed: {:?}", result.err());
    println!("✓ RFM95 initialized successfully");
}

#[test]
#[ignore]
fn test_rfm95_version_check() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    let result = rfm.reset();
    assert!(
        result.is_ok(),
        "Reset should succeed (version 0x12 detected)"
    );
    println!("✓ Version check passed (0x12 detected)");
}

#[test]
#[ignore]
fn test_rfm95_mode_transitions() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");

    let result = rfm.enter_receive_mode(FREQ_CH0, DataRate::SF7BW125);
    assert!(
        result.is_ok(),
        "Failed to enter receive mode: {:?}",
        result.err()
    );
    println!("✓ Mode transitions working correctly");
}

#[test]
#[ignore]
fn test_rfm95_frequency_setting() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");

    let frequencies = [FREQ_CH0, lora_rfm98::FREQ_CH1, lora_rfm98::FREQ_CH2];

    for freq in &frequencies {
        let result = rfm.enter_receive_mode(*freq, DataRate::SF7BW125);
        assert!(
            result.is_ok(),
            "Failed to set frequency {:?}: {:?}",
            freq,
            result.err()
        );
        thread::sleep(Duration::from_millis(50));
    }
    println!("✓ Frequency settings work correctly");
}

#[test]
#[ignore]
fn test_rfm95_data_rate_configurations() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");

    let data_rates = [DataRate::SF7BW125, DataRate::SF8BW125, DataRate::SF9BW125];

    for dr in &data_rates {
        let result = rfm.enter_receive_mode(FREQ_CH0, *dr);
        assert!(
            result.is_ok(),
            "Failed to set data rate {:?}: {:?}",
            dr,
            result.err()
        );
        thread::sleep(Duration::from_millis(50));
    }
    println!("✓ All data rate configurations successful");
}

#[test]
#[ignore]
fn test_rfm95_multiple_resets() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");

    for i in 0..3 {
        let result = rfm.reset();
        assert!(result.is_ok(), "Reset {} failed: {:?}", i + 1, result.err());
        thread::sleep(Duration::from_millis(100));
    }
    println!("✓ Multiple resets successful");
}

#[test]
#[ignore]
fn test_rfm95_dio0_status() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");

    let dio0_status = rfm.is_dio0_high();
    assert!(
        dio0_status.is_ok(),
        "Failed to read DIO0 status: {:?}",
        dio0_status.err()
    );
    println!("✓ DIO0 status: {:?}", dio0_status.unwrap());
}

#[test]
#[ignore]
fn test_rfm95_receive_mode_entry() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");

    let result = rfm.enter_receive_mode(FREQ_CH0, DataRate::SF7BW125);
    assert!(
        result.is_ok(),
        "Failed to enter receive mode: {:?}",
        result.err()
    );
    thread::sleep(Duration::from_millis(100));

    let dio0 = rfm.is_dio0_high().expect("Failed to read DIO0");
    println!("✓ Receive mode entered, DIO0 state: {}", dio0);
}

#[test]
#[ignore]
fn test_rfm95_packet_size_validation() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");

    let empty_packet = [];
    let result = rfm.send_packet(&empty_packet, FREQ_CH0, DataRate::SF7BW125);
    assert!(result.is_err(), "Empty packet should be rejected");
    assert_eq!(result.unwrap_err(), RFMError::InvalidPacketSize);

    let oversized_packet = [0u8; 256];
    let result = rfm.send_packet(&oversized_packet, FREQ_CH0, DataRate::SF7BW125);
    assert!(result.is_err(), "Oversized packet should be rejected");
    assert_eq!(result.unwrap_err(), RFMError::InvalidPacketSize);

    println!("✓ Packet size validation working correctly");
}

#[test]
#[ignore]
fn test_with_led_indicator() {
    let gpio = Gpio::new().expect("Failed to initialize GPIO");
    let mut led = gpio
        .get(PIN_LED)
        .expect("Failed to get LED pin")
        .into_output();

    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");

    led.set_high();
    thread::sleep(Duration::from_millis(500));

    let result = rfm.reset();

    if result.is_ok() {
        for _ in 0..5 {
            led.set_low();
            thread::sleep(Duration::from_millis(100));
            led.set_high();
            thread::sleep(Duration::from_millis(100));
        }
        println!("✓ Test passed - LED indicator");
    } else {
        led.set_high();
        panic!("Test failed: {:?}", result.err());
    }

    led.set_low();
}

#[test]
#[ignore]
fn test_pin_connectivity() {
    println!("Testing pin connectivity...");

    let gpio = Gpio::new().expect("Failed to initialize GPIO");

    let mut reset = gpio
        .get(PIN_RESET)
        .expect("Failed to get reset pin")
        .into_output();
    reset.set_high();
    reset.set_low();
    println!("  ✓ Reset pin (GPIO{}) working", PIN_RESET);

    let dio0 = gpio
        .get(PIN_DIO0)
        .expect("Failed to get DIO0 pin")
        .into_input();
    println!(
        "  ✓ DIO0 pin (GPIO{}) working, current state: {}",
        PIN_DIO0,
        dio0.is_high()
    );

    let mut led = gpio
        .get(PIN_LED)
        .expect("Failed to get LED pin")
        .into_output();
    led.set_high();
    thread::sleep(Duration::from_millis(500));
    led.set_low();
    println!("  ✓ LED pin (GPIO{}) working", PIN_LED);

    let _spi =
        Spi::new(SPI_BUS, SPI_CS, 10_000_000, Mode::Mode0).expect("Failed to initialize SPI");
    println!("  ✓ SPI initialized (MOSI=GPIO10, MISO=GPIO9, SCLK=GPIO11, CS=GPIO7)");

    println!("✓ All pin connections verified");
}

#[test]
#[ignore]
fn test_rfm95_stress_mode_changes() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");

    println!("Running stress test: 50 rapid mode changes...");

    for i in 0..50 {
        let dr = match i % 3 {
            0 => DataRate::SF7BW125,
            1 => DataRate::SF8BW125,
            _ => DataRate::SF9BW125,
        };

        let freq = match i % 3 {
            0 => FREQ_CH0,
            1 => lora_rfm98::FREQ_CH1,
            _ => lora_rfm98::FREQ_CH2,
        };

        let result = rfm.enter_receive_mode(freq, dr);
        assert!(
            result.is_ok(),
            "Failed at iteration {}: {:?}",
            i,
            result.err()
        );

        if i % 10 == 0 {
            println!("  Progress: {}/50", i);
        }
    }

    println!("✓ Stress test completed successfully");
}

#[test]
#[ignore]
fn test_rfm95_dio0_monitoring() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");
    rfm.reset().expect("Reset failed");
    rfm.enter_receive_mode(FREQ_CH0, DataRate::SF7BW125)
        .expect("Failed to enter RX mode");

    println!("Monitoring DIO0 for 5 seconds...");

    let mut high_count = 0;
    let mut low_count = 0;

    for i in 0..50 {
        let is_high = rfm.is_dio0_high().expect("Failed to read DIO0");
        if is_high {
            high_count += 1;
        } else {
            low_count += 1;
        }

        if i % 10 == 0 {
            println!(
                "  Sample {}: DIO0 = {}",
                i,
                if is_high { "HIGH" } else { "LOW" }
            );
        }

        thread::sleep(Duration::from_millis(100));
    }

    println!(
        "✓ DIO0 monitoring complete: {} high, {} low",
        high_count, low_count
    );
}

#[test]
#[ignore]
fn test_rfm95_error_recovery() {
    let mut rfm = create_rfm95().expect("Failed to create RFM95 instance");

    println!("Testing error recovery...");

    rfm.reset().expect("Initial reset failed");

    let empty = [];
    let _ = rfm.send_packet(&empty, FREQ_CH0, DataRate::SF7BW125);

    let result = rfm.enter_receive_mode(FREQ_CH0, DataRate::SF7BW125);
    assert!(
        result.is_ok(),
        "Failed to recover after error: {:?}",
        result.err()
    );

    println!("✓ Error recovery successful");
}

