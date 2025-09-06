#![no_main]
#![no_std]

use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::Peripherals;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::UartTx;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    usart::{self, Config, DataBits, Parity, StopBits, Uart},
};
use embassy_time::Timer;
use embedded_graphics::Drawable;
use embedded_graphics::text::Baseline;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::Text,
};
use heapless::String;
use panic_probe as _;
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::DisplayConfig, size::DisplaySize128x32};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

fn send_uart_message<const N: usize>(
    tx: &mut UartTx<'_, embassy_stm32::mode::Blocking>,
    message: &String<N>,
) -> Result<(), embassy_stm32::usart::Error> {
    tx.blocking_write(message.as_bytes())?;
    tx.blocking_flush()?;
    Ok(())
}

macro_rules! uart_println {
    ($tx:expr, $($arg:tt)*) => {
        {
            let mut buffer: String<256> = String::new();
            write!(buffer, $($arg)*).ok();
            buffer.push_str("\r\n").ok();
            send_uart_message($tx, &buffer)
        }
    };
}

fn setup_peripherals() -> Peripherals {
    use embassy_stm32::rcc::*;

    let mut periph_config = embassy_stm32::Config::default();

    periph_config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(16),
        mode: HseMode::Oscillator,
    });
    periph_config.rcc.pll = Some(Pll {
        src: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL9,
    });
    periph_config.rcc.sys = Sysclk::PLL1_P;
    periph_config.rcc.ahb_pre = AHBPrescaler::DIV1;
    periph_config.rcc.apb1_pre = APBPrescaler::DIV2;
    periph_config.rcc.apb2_pre = APBPrescaler::DIV1;

    embassy_stm32::init(periph_config)
}

async fn setup_io(p: Peripherals) -> (UartTx<'static, Blocking>, I2c<'static, Blocking>) {
    let mut led = Output::new(p.PA7, Level::Low, Speed::Low);

    let mut config = Config::default();
    config.baudrate = 115200;
    config.stop_bits = StopBits::STOP1;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityNone;

    // Peripherals must be remapped for custom PCB pinout!!!
    embassy_stm32::pac::AFIO
        .mapr()
        .modify(|w| w.set_usart1_remap(true));

    let usart_result = Uart::new_blocking(p.USART1, p.PB7, p.PB6, config);

    let usart = match usart_result {
        Ok(u) => u,
        Err(_) => loop {
            led.set_high();
            embassy_time::Timer::after_millis(500).await;
            led.set_low();
            embassy_time::Timer::after_millis(500).await;
        },
    };

    let (tx, _rx) = usart.split();

    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz::khz(100), Default::default());

    (tx, i2c)
}

async fn check_i2c_device(i2c: &mut I2c<'static, Blocking>, tx: &mut UartTx<'static, Blocking>) {
    if let Ok(_) = i2c.blocking_write(0x3C, &[]) {
        uart_println!(tx, "Found I2C device at: 0x{:02X}", 0x3C).unwrap();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = setup_peripherals();

    let (mut tx, mut i2c) = setup_io(p).await;

    uart_println!(&mut tx, "===Program Start===").unwrap();

    check_i2c_device(&mut i2c, &mut tx).await;

    let interface = I2CDisplayInterface::new_custom_address(i2c, 0x3C);

    let mut display = Ssd1306::new(
        interface,
        DisplaySize128x32,
        ssd1306::prelude::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    Timer::after_millis(100).await;

    if let Err(e) = display.init() {
        loop {
            uart_println!(&mut tx, "Error: {:?}", e).unwrap();
            Timer::after_secs(3).await;
        }
    };

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        "Systems Normal!",
        Point::new(0, 0),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    display.flush().unwrap();

    loop {
        uart_println!(&mut tx, "Program Running!").unwrap();
        Timer::after_secs(2).await;
    }
}
