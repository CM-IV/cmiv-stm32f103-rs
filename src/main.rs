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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Init");

    let p = embassy_stm32::init(Default::default());

    let mut led = Output::new(p.PA6, Level::Low, Speed::Low);

    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz(50_000), Default::default());

    begin_lcd_commands(i2c, &mut led).await;

    loop {
        info!("Running!");

        led.set_high();

        Timer::after_millis(3000).await;

        led.set_low();

        Timer::after_millis(3000).await;
    }
}

async fn begin_lcd_commands(i2c: I2c<'static, Blocking>, led: &mut Output<'static>) {
    let mut lcd =
        CharacterDisplayPCF8574T::new_with_address(i2c, 0x27, LcdDisplayType::Lcd16x2, Delay);

    match lcd.init() {
        Ok(_) => {
            // Commands go in here vvv
            if let Err(_) = lcd.backlight(true) {
                error!("Error starting backlight");
            }
            if let Err(_) = lcd.clear() {
                error!("Error clearing LCD");
            }
            if let Err(_) = lcd.home() {
                error!("Error setting cursor to home");
            }

            if let Err(_) = lcd.print("Hello Charlie!") {
                error!("Error printing!");
            }

            if let Err(_) = lcd.show_cursor(true) {
                error!("Error showing cursor!");
            }

            if let Err(_) = lcd.blink_cursor(true) {
                error!("Error blinking cursor!");
            }
        }
        Err(_) => {
            led.set_high();

            Timer::after_millis(1000).await;

            led.set_low();

            Timer::after_millis(1000).await;
        }
    }
}
