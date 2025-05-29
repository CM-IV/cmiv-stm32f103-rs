#![deny(unsafe_code)]
#![no_main]
#![no_std]

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

#[derive(Debug)]
enum AppError {
    LcdCommandFailed,
    LcdInitFailed,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut led = Output::new(p.PA7, Level::Low, Speed::Low);

    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz(50_000), Default::default());

    if run_main_loop(i2c).await.is_err() {
        blink_error_led(&mut led).await;
    }
}

async fn run_main_loop(i2c: I2c<'static, Blocking>) -> Result<(), AppError> {
    let mut lcd =
        CharacterDisplayPCF8574T::new_with_address(i2c, 0x27, LcdDisplayType::Lcd16x2, Delay);

    lcd.init().map_err(|_| AppError::LcdInitFailed)?;

    loop {
        lcd.backlight(true)
            .map_err(|_| AppError::LcdCommandFailed)?;
        lcd.clear().map_err(|_| AppError::LcdCommandFailed)?;
        lcd.home().map_err(|_| AppError::LcdCommandFailed)?;
        lcd.print("Hello Charlie...")
            .map_err(|_| AppError::LcdCommandFailed)?;
        lcd.set_cursor(0, 1)
            .map_err(|_| AppError::LcdCommandFailed)?;
        lcd.print("Test ").map_err(|_| AppError::LcdCommandFailed)?;
        lcd.print(itoa::Buffer::new().format(123))
            .map_err(|_| AppError::LcdCommandFailed)?;

        Timer::after_secs(4).await;

        lcd.clear().map_err(|_| AppError::LcdCommandFailed)?;
        lcd.home().map_err(|_| AppError::LcdCommandFailed)?;
        lcd.print("Second screen")
            .map_err(|_| AppError::LcdCommandFailed)?;
        lcd.set_cursor(0, 1)
            .map_err(|_| AppError::LcdCommandFailed)?;
        lcd.print("Counter: ")
            .map_err(|_| AppError::LcdCommandFailed)?;
        lcd.print(itoa::Buffer::new().format(456))
            .map_err(|_| AppError::LcdCommandFailed)?;

        Timer::after_secs(4).await;
    }
}

async fn blink_error_led(led: &mut Output<'static>) -> ! {
    loop {
        led.set_high();
        Timer::after_millis(1000).await;
        led.set_low();
        Timer::after_millis(1000).await;
    }
}
