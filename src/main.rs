#![no_main]
#![no_std]

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::asm::nop;
use embassy_executor::{Spawner, task};
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
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Timer};
use embedded_graphics::Drawable;
use embedded_graphics::text::Baseline;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::Text,
};
use embedded_hal_bus::i2c;
use heapless::String;
use panic_probe as _;
use sht31::SHT31;
use sht31::mode::Sht31Reader;
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::DisplayConfig, size::DisplaySize128x32};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

static UART_CHANNEL: Channel<ThreadModeRawMutex, String<256>, 10> = Channel::new();

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

#[task]
async fn uart_writer(mut tx: UartTx<'static, Blocking>) {
    loop {
        let message = UART_CHANNEL.receive().await;
        send_uart_message(&mut tx, &message).ok();
    }
}

#[task]
async fn start(i2c: I2c<'static, Blocking>) {
    let mut msg: String<256> = String::new();

    let i2c_refcell = RefCell::new(i2c);
    let i2c_device1 = i2c::RefCellDevice::new(&i2c_refcell);
    let i2c_device2 = i2c::RefCellDevice::new(&i2c_refcell);

    let interface = I2CDisplayInterface::new_custom_address(i2c_device1, 0x3C);

    let mut display = Ssd1306::new(
        interface,
        DisplaySize128x32,
        ssd1306::prelude::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    Timer::after_millis(200).await;

    if let Err(e) = display.init() {
        loop {
            write!(&mut msg, "Display Error: {:?}\r\n", e).ok();
            UART_CHANNEL.send(msg.clone()).await;
            Timer::after_secs(3).await;
        }
    };

    write!(&mut msg, "===Program Start===\r\n").ok();
    UART_CHANNEL.send(msg.clone()).await;
    msg.clear();

    write!(&mut msg, "===Display OK!===\r\n").ok();
    UART_CHANNEL.send(msg.clone()).await;
    msg.clear();

    let mut sht = SHT31::new(i2c_device2, Delay);

    write!(&mut msg, "===Temp Setup===\r\n").ok();
    UART_CHANNEL.send(msg.clone()).await;
    msg.clear();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    loop {
        match sht.read() {
            Ok(reading) => {
                display.clear_buffer();

                msg.clear();
                write!(&mut msg, "Temp: {:.1}F", reading.temperature).unwrap();

                msg.clear();
                write!(&mut msg, "Temperature: {:.1}F\r\n", reading.temperature).ok();
                UART_CHANNEL.send(msg.clone()).await;

                Text::with_baseline(
                    "Systems Normal",
                    Point::new(0, 0),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                Text::with_baseline(&msg, Point::new(0, 16), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                display.flush().unwrap();
            }
            Err(_) => {
                msg.clear();
                msg.push_str("Sensor Error\r\n").ok();
                UART_CHANNEL.send(msg.clone()).await;
            }
        }

        Timer::after_secs(2).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = setup_peripherals();

    let (tx, i2c) = setup_io(p).await;

    spawner.spawn(uart_writer(tx)).ok();
    spawner.spawn(start(i2c)).ok();

    loop {
        nop();
        Timer::after_secs(10).await;
    }
}
