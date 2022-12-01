#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use gd32vf103xx_hal::gpio::gpioa::PA3;
use gd32vf103xx_hal::gpio::{OpenDrain, Output};
use heapless::String;

use embedded_graphics::mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;

use longan_nano::hal::delay::McycleDelay;
use longan_nano::hal::{pac, prelude::*};
use longan_nano::{lcd, lcd_pins};

use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcu = p
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();
    let mut afio = p.AFIO.constrain(&mut rcu);
    let mut del = McycleDelay::new(&rcu.clocks);
    let gpioa = p.GPIOA.split(&mut rcu);
    let gpiob = p.GPIOB.split(&mut rcu);

    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(p.SPI0, lcd_pins, &mut afio, &mut rcu);
    let (width, height) = (lcd.size().width as i32, lcd.size().height as i32);

    //create static buffer for formatting data
    let mut buffer = String::<20>::from("");

    // Clear screen
    Rectangle::new(Point::new(0, 0), Size::new(width as u32, height as u32))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut lcd)
        .unwrap();
    let mut dht_pin = gpioa.pa3.into_open_drain_output();
    dht_pin.set_high().ok();
    del.delay_ms(1000_u16);

    loop {
        let dht_data = start_probe(&mut dht_pin, del, &mut lcd);
        let checksum = dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3];
        if checksum == dht_data[4] {
            let _ = write!(buffer, "HUM: {}.{}%", dht_data[0], dht_data[1]);
            lcd_print(&mut lcd, 20, 20, buffer.as_str());
            buffer.clear();
            let _ = write!(buffer, "TEMP: {}.{}C", dht_data[2], dht_data[3]);
            lcd_print(&mut lcd, 20, 40, buffer.as_str());
            buffer.clear();
        } else {
            lcd_print(&mut lcd, 5, 30, "-- Checksum Error!");
        }
    }
}

fn start_probe(
    dht_pin: &mut PA3<Output<OpenDrain>>,
    mut delay: McycleDelay,
    lcd: &mut lcd::Lcd,
) -> ([u8; 5]) {
    dht_pin.set_low().ok(); // bus down, send start signal
    delay.delay_ms(30_u8); // delay greater than 18ms, so DHT11 start signal can be detected
    dht_pin.set_high().ok();
    delay.delay_us(40_u8); // Wait for DHT11 response

    while dht_pin.is_high().unwrap() {}
    delay.delay_us(80_u8); //The DHT11 responds by pulling the bus low for 80us
    while dht_pin.is_low().unwrap() {}
    delay.delay_us(80_u8); //DHT11 pulled up after the bus 80us to start sending data

    let mut data = [0; 5];
    //Receiving temperature and humidity data, check bits are not considered
    for i in data.iter_mut() {
        *i = read_data(dht_pin, delay, lcd);
    }
    dht_pin.set_high().unwrap(); //After the completion of a release of data bus, waiting for the host to start the next signal
    data
}

fn read_data(
    dht_pin: &mut PA3<Output<OpenDrain>>,
    mut delay: McycleDelay,
    _lcd: &mut lcd::Lcd,
) -> u8 {
    let mut byte = 0;
    for i in 0..8 {
        let bit_mask = 1 << (7 - (i % 8));
        while dht_pin.is_low().unwrap() {} // wait 50us
        delay.delay_us(30_u8);
        if dht_pin.is_high().unwrap() {
            byte |= bit_mask; //High in the former, low in the post
        }
        while dht_pin.is_high().unwrap() {} //Data '1', waiting for the next bit of reception'
    }
    // let mut buffer = String::<20>::from("");
    // let _ = write!(buffer, "{} {}",byte,dht_pin.is_high().unwrap());
    // lcd_print(lcd, 40, 55, buffer.as_str());
    // buffer.clear();
    byte
}

fn lcd_print(lcd: &mut lcd::Lcd, xpos: i32, ypos: i32, text: &str) {
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    // Create a text at position (x, y) and draw it using style defined above
    Text::new(text, Point::new(xpos, ypos), style)
        .draw(lcd)
        .unwrap();
}
