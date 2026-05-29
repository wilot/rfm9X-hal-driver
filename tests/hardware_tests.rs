use linux_embedded_hal::gpio_cdev::{Chip, LineRequestFlags};
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::{CdevPin, Delay, SpidevDevice};
use lora_rfm98::{DataRate, FskDataRate, RFM95, RFMError, FREQ_CH0, FREQ_CH1};

const SPI_DEV_PATH: &str = "/dev/spidev0.0";
const GPIO_CHIP_PATH: &str = "/dev/gpiochip0";
const PIN_RESET: u32 = 12;
const PIN_DIO0: u32 = 16;

fn request_output_pin(pin: u32) -> Result<CdevPin, Box<dyn std::error::Error>> {
    let mut chip = Chip::new(GPIO_CHIP_PATH)?;
    let line = chip.get_line(pin)?;
    let handle = line.request(LineRequestFlags::OUTPUT, 1, "rfm9x-hal-tests")?;
    Ok(CdevPin::new(handle)?)
}

fn request_input_pin(pin: u32) -> Result<CdevPin, Box<dyn std::error::Error>> {
    let mut chip = Chip::new(GPIO_CHIP_PATH)?;
    let line = chip.get_line(pin)?;
    let handle = line.request(LineRequestFlags::INPUT, 0, "rfm9x-hal-tests")?;
    Ok(CdevPin::new(handle)?)
}

fn create_rfm95() -> Result<RFM95<SpidevDevice, CdevPin, CdevPin, Delay>, Box<dyn std::error::Error>>
{
    let mut spi = SpidevDevice::open(SPI_DEV_PATH)?;
    let spi_cfg = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(10_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&spi_cfg)?;

    let reset = request_output_pin(PIN_RESET)?;
    let dio0 = request_input_pin(PIN_DIO0)?;
    let delay = Delay;

    Ok(RFM95::new(spi, reset, dio0, delay))
}

#[test]
#[ignore]
fn lora_mode_transition_works() {
    let mut rfm = create_rfm95().expect("failed to initialize linux-hal RFM95");
    rfm.reset().expect("reset failed");
    rfm.enter_receive_mode(FREQ_CH0, DataRate::SF7BW125)
        .expect("failed to enter LoRa receive mode");
}

#[test]
#[ignore]
fn fsk_mode_transition_works() {
    let mut rfm = create_rfm95().expect("failed to initialize linux-hal RFM95");
    rfm.reset().expect("reset failed");
    rfm.enter_fsk_receive_mode(FREQ_CH1, FskDataRate::BR50kFd25k)
        .expect("failed to enter FSK receive mode");
}

#[test]
#[ignore]
fn packet_size_validation_applies_to_lora_and_fsk() {
    let mut rfm = create_rfm95().expect("failed to initialize linux-hal RFM95");
    rfm.reset().expect("reset failed");

    let empty: [u8; 0] = [];
    assert_eq!(
        rfm.send_packet(&empty, FREQ_CH0, DataRate::SF7BW125),
        Err(RFMError::InvalidPacketSize)
    );
    assert_eq!(
        rfm.send_fsk_packet(&empty, FREQ_CH1, FskDataRate::BR100kFd50k),
        Err(RFMError::InvalidPacketSize)
    );
}
