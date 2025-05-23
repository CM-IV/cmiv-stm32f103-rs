#![deny(unsafe_code)]
#![no_main]
#![no_std]

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    i2c::I2c,
    mode::Blocking,
    time::Hertz,
};
use embassy_time::{Delay, Timer};
use i2c_character_display::{CharacterDisplayPCF8574T, LcdDisplayType};
use panic_probe as _;

#[derive(Debug, defmt::Format)]
enum LcdError {
    CommandFailed,
    InitFailed,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Init");

    let p = embassy_stm32::init(Default::default());

    let mut led = Output::new(p.PA6, Level::Low, Speed::Low);

    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz(50_000), Default::default());

    match begin_lcd_commands(i2c).await {
        Ok(_) => info!("LCD initialized successfully"),
        Err(e) => {
            error!("LCD error: {:?}", e);
            loop {
                led.set_high();
                Timer::after_millis(1000).await;
                led.set_low();
                Timer::after_millis(1000).await;
            }
        }
    }
}

async fn begin_lcd_commands(i2c: I2c<'static, Blocking>) -> Result<(), LcdError> {
    let mut lcd =
        CharacterDisplayPCF8574T::new_with_address(i2c, 0x27, LcdDisplayType::Lcd16x2, Delay);

    lcd.init().map_err(|_| {
        error!("LCD init failed");
        LcdError::InitFailed
    })?;

    // Commands go in here vvv
    lcd.backlight(true).map_err(|_| LcdError::CommandFailed)?;
    lcd.clear().map_err(|_| LcdError::CommandFailed)?;
    lcd.home().map_err(|_| LcdError::CommandFailed)?;
    lcd.print("Hello Charlie!")
        .map_err(|_| LcdError::CommandFailed)?;
    lcd.blink_cursor(true)
        .map_err(|_| LcdError::CommandFailed)?;

    Ok(())
}
