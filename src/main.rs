// #![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;

#[allow(unused_imports)]
use panic_semihosting;

use rtt_target::{rprintln, rtt_init_print};

use stm32f7xx_hal::{
    pac,
    prelude::*,
    rcc::{HSEClock, HSEClockMode, Rcc},
};

mod touch;

// fn draw_keypad(display: &mut Stm32F7DiscoDisplay<u16>) {
//     let c = BACKGROUND_COLOR;
//     let background_color: u32 =
//         c.b() as u32 & 0x1F | ((c.g() as u32 & 0x3F) << 5) | ((c.r() as u32 & 0x1F) << 11);

//     unsafe {
//         display
//             .controller
//             .draw_rectangle(Layer::L1, (0, 0), (480, 272), background_color);
//     }

//     for i in (0..3).rev() {
//         for j in 0..3 {
//             let a = match 1 + i * 3 + j {
//                 0 => "0",
//                 1 => "1",
//                 2 => "2",
//                 3 => "3",
//                 4 => "4",
//                 5 => "5",
//                 6 => "6",
//                 7 => "7",
//                 8 => "8",
//                 9 => "9",
//                 _ => "",
//             };
//             button(
//                 KEY_X_OFFSET + j * KEY_X_SPACING,
//                 KEY_Y_OFFSET + i * KEY_Y_SPACING,
//                 a,
//                 display,
//             );
//         }
//     }
//     button(KEY_X_OFFSET, KEY_Y_OFFSET + 3 * KEY_Y_SPACING, "0", display);
//     button(
//         KEY_X_OFFSET + 1 * KEY_X_SPACING,
//         KEY_Y_OFFSET + 3 * KEY_Y_SPACING,
//         ".",
//         display,
//     );
//     button(
//         KEY_X_OFFSET + 2 * KEY_X_SPACING,
//         KEY_Y_OFFSET + 3 * KEY_Y_SPACING,
//         "",
//         display,
//     );

//     let style = PrimitiveStyleBuilder::new()
//         .stroke_width(BUTTON_STROKE_WIDTH)
//         .stroke_color(BUTTON_STROKE_COLOR)
//         .fill_color(Rgb565::BLACK)
//         .build();

//     RoundedRectangle::with_equal_corners(
//         Rectangle::new(
//             Point::new(KEY_X_OFFSET, KEY_Y_OFFSET - 55),
//             Size::new(BUTTON_WIDTH + (2 * KEY_X_SPACING) as u32, BUTTON_HEIGHT),
//         ),
//         Size::new(CORNER_RADIUS, CORNER_RADIUS),
//     )
//     .into_styled(style)
//     .draw(display)
//     .ok();

//     let style = MonoTextStyle::new(&PROFONT_24_POINT, Rgb565::YELLOW);

//     Text::new(
//         "300.89",
//         Point::new(KEY_X_OFFSET + TEXT_XOFFSET, 5 + TEXT_YOFFSET),
//         style,
//     )
//     .draw(display)
//     .ok();
// }

// impl cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs<i32> for stm32f7xx_hal::delay::Delay {
//     fn delay(&self) {}
// }

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Started");

    let perif = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc: Rcc = perif.RCC.constrain();

    let clocks = rcc
        .cfgr
        .hse(HSEClock::new(25_000_000.Hz(), HSEClockMode::Bypass))
        .sysclk(216_000_000.Hz())
        .hclk(216_000_000.Hz())
        .freeze();
    // let mut delay = Delay::new(cp.SYST, clocks);

    rprintln!("Connecting to I2c");
    let gpioh = perif.GPIOH.split();
    let scl = gpioh.ph7.into_alternate_open_drain::<4>(); //LCD_SCL
    let sda = gpioh.ph8.into_alternate_open_drain::<4>(); //LSD_SDA

    let touch = touch::Touch::new(perif.I2C3, scl, sda, &mut rcc.apb1, clocks);

    touch.test();

    loop {}
}

// }

// This works in main file:
/*
    fn use_pins(
        i2c_port: I2C3,
        scl: stm32f7xx_hal::gpio::Pin<AlternateOD<4_u8>, 'H', 7_u8>,
        sda: stm32f7xx_hal::gpio::Pin<AlternateOD<4_u8>, 'H', 8_u8>,
        mode: Mode,
        apb1: &mut APB1,
        clocks: Clocks,
    ) {
        let mut i2c = BlockingI2c::i2c3(i2c_port, (scl, sda), mode, clocks, apb1, 10_000);

        let cmd: [u8; 1] = [0];
        let mut buf: [u8; 1] = [0];

        if i2c.write(FT5336_TOUCHPAD_ADDR, &cmd).is_ok() {
            rprintln!("Wrote to touchpad okay");
        }
    }
*/

// let tp = touch::Touchpad(sda: SDA, scl: SCL, i2c: BlockingI2c<I2C3, SCL, SDA>);

// let mut i2c = BlockingI2c::i2c3(
//     perif.I2C3,
//     (scl, sda),
//     Mode::fast(100_000.Hz()),
//     clocks,
//     &mut rcc.apb1,
//     10_000,
// );

/*
fn standby() {
    // let tp = touch::Touch::new(i2c);
    // let tp = touch::Touch::new(i2c, touch::FT5336_TOUCHPAD_ADDR, 10000);

    // const VALID_ADDR_RANGE: Range<u8> = 0x08..0x78;

    // ****************************************************************************************************

    // for addr in 0x00_u8..0x80 {
    //     // Write the empty array and check the slave response.
    //     let byte: [u8; 1] = [0; 1];
    //     if VALID_ADDR_RANGE.contains(&addr) && i2c.write(addr, &byte).is_ok() {
    //         addresses += 1;
    //         rprintln!("Address: {}", addr);
    //     }
    // }
    // rprintln!("Found {} I2C devices on bus 3", addresses);

    let cmd: [u8; 1] = [0];
    let mut buf: [u8; 1] = [0];

    const FT5336_OK: u8 = 0;
    const FT5336_ERROR: u8 = 0xff;
    // const FT5336_MAX_NB_TOUCH: u8 = 0x05;
    // I2C device addresses
    const FT5336_DEV_MODE_REG: u8 = 0x00;

    /* Gesture ID register */
    const FT5336_GEST_ID_REG: u8 = 0x01;

    /* Touch Data Status register : gives number of active touch points (0..2) */
    const FT5336_TD_STAT_REG: u8 = 0x02;

    /* P1 X, Y coordinates, weight and misc registers */
    const FT5336_P1_XH_REG: u8 = 0x03;
    const FT5336_P1_XL_REG: u8 = 0x04;
    const FT5336_P1_YH_REG: u8 = 0x05;
    const FT5336_P1_YL_REG: u8 = 0x06;
    const FT5336_P1_WEIGHT_REG: u8 = 0x07;
    const FT5336_P1_MISC_REG: u8 = 0x08;
    const FT5336_P1_XH_TP_BIT_MASK: u8 = 0x0F;
    const FT5336_P1_XH_TP_BIT_POSITION: u8 = 0;
    const FT5336_P1_XL_TP_BIT_MASK: u8 = 0xFF;
    const FT5336_P1_XL_TP_BIT_POSITION: u8 = 0;

    const FT5336_GMODE_REG: u8 = 0xA4;

    /* FT5336 Chip identification register */
    const FT5336_CHIP_ID_REG: u8 = 0xA8;

    /* Release code version */
    const FT5336_RELEASE_CODE_ID_REG: u8 = 0xAF;

    /* Current operating mode the FT5336 system is in (R) */
    const FT5336_STATE_REG: u8 = 0xBC;

    // loop {}



    // if !i2c
    //     .write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_CHIP_ID_REG], &mut buf)
    //     .is_ok()
    // {
    //     rprintln!("Attempt to read chip ID Failed");
    // } else {
    //     let v = buf[0] as u8;
    //     rprintln!("Chip id is {:02x}", v);
    // }

    // if !i2c
    //     .write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_STATE_REG], &mut buf)
    //     .is_ok()
    // {
    //     rprintln!("Attempt to read chip state failed");
    // } else {
    //     let v = buf[0] as u8;
    //     rprintln!("Chip state is {:02x}", v);
    // }

    // let mut buf: [u8; 2] = [0, 0];
    // if !i2c
    //     .write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_XH_REG], &mut buf)
    //     .is_ok()
    // {
    //     rprintln!("Attempt to read P1_XH failed");
    // } else {
    //     let v = buf[0] as u8;
    //     let w = buf[1] as u8;
    //     rprintln!("Chip state is {:02x}, {:02x}", v, w);
    // }

    let mut step_number: u32 = 0;
    loop {
        let mut buf: [u8; 1] = [0];
        if !i2c
            .write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_TD_STAT_REG], &mut buf)
            .is_ok()
        {
            rprintln!("Attempt to read chip status failed");
        } else {
            if buf[0] == 1 {
                let _v = buf[0] as u8;
                let mut buf: [u8; 5] = [0; 5];
                i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_XH_REG], &mut buf);
                // let mut x1 = buf[0] as u8;
                // i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_XL_REG], &mut buf);
                // let x2 = buf[0] as u8;
                // i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_YH_REG], &mut buf);
                // let mut y1 = buf[0] as u8;
                // i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_YL_REG], &mut buf);
                // let y2 = buf[0] as u8;
                //
                // if v != 255 && v != 0 {
                //     rprintln!(
                //         "Touch status is {:02x} - Key is {:02x}{:02x}x{:02x}{:02x}",
                //         v,
                //         buf[0] as u8,
                //         buf[1] as u8,
                //         buf[2] as u8,
                //         buf[3] as u8,
                //     );
                // }
                let status: [u8; 1] = [0];
                let a = i2c
                    .write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_GMODE_REG], &mut buf)
                    .is_ok();
                // rprintln!("Status is: {:02x}", status[0]);
                rprintln!(
                    "{:10}: {:03},{:03}  - {:02x}",
                    step_number,
                    buf[2] as u16 * 256 + buf[3] as u16,
                    269 - ((buf[0] & 0x7F) as u16 * 256 + buf[1] as u16),
                    buf[4] as u8,
                );
                // let mut buf: [u8; 1] = [0];
                // i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_XH_REG], &mut buf);
                // let mut xh = buf[0] as u8;
                // i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_XL_REG], &mut buf);
                // let xl = buf[0] as u8;
                // i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_YH_REG], &mut buf);
                // let mut yh = buf[0] as u8;
                // i2c.write_read(FT5336_TOUCHPAD_ADDR, &[FT5336_P1_YL_REG], &mut buf);
                // let yl = buf[0] as u8;
                // if v != 255 && v != 0 {
                //     rprintln!(
                //         "------------ is {:02x} - Key is {:02x}{:02x}x{:02x}{:02x}",
                //         v,
                //         xh as u8,
                //         xl as u8,
                //         yh as u8,
                //         yl as u8,
                //     );
                // }
            } else {
                rprintln!("fingers: {}", buf[0]);
            }
        }
        // delay.delay_us(500);
        step_number += 1;
    }

    // const FT62XX_REG_NUMTOUCHES: u8 = 0x0; // !< Touch X position

    // let byte: [u8; 1] = [0; 1]; //FT62XX_REG_NUMTOUCHES; 1];
    // let mut buffer: [u8; 16] = [0; 16];
    // if !i2c.write(0x1A, &byte).is_ok() {
    //     rprintln!("Error response when sending request for touches");
    // } else {
    //     if !i2c.read(0x38, &mut buffer).is_ok() {
    //         rprintln!("Error in reading for touches");
    //     };
    //     for i in 0..16 {
    //         rprintln!("{}: {}", i, buffer[i] as u8);
    //     }
    // }

    // let ft5336 = Ft5336::new(i2c);

    // rprintln!("On 0x38 the registers contain:");
    // let mut byte: [u8; 10] = [0x01; 10];
    // for i in 0x00_u8..0x10 {
    //     if i2c.write(0x38, &byte).is_ok() {
    //         i2c.read(0x38, &mut byte);
    //         for i in 0..10 {
    //             rprintln!("Buffer {}: {}", i, byte[i] as u8);
    //         }
    //     } else {
    //         rprintln!("unsuccessful at writing");
    //     }
    // }

    // let style = MonoTextStyle::new(&PROFONT_24_POINT, Rgb565::YELLOW);

    // let a = match addresses {
    //     0 => "0",
    //     1 => "1",
    //     2 => "2",
    //     3 => "3",
    //     4 => "4",
    //     5 => "5",
    //     6 => "6",
    //     7 => "7",
    //     8 => "8",
    //     9 => "9",
    //     _ => "More",
    // };

    // const FT62XX_REG_NUMTOUCHES: u8 = 0x0; // !< Touch X position

    // let byte: [u8; 1] = [0; 1]; //FT62XX_REG_NUMTOUCHES; 1];
    // let mut buffer: [u8; 16] = [0; 16];
    // if !i2c.write(0x1A, &byte).is_ok() {
    //     rprintln!("Error response when sending request for touches");
    // } else {
    //     if !i2c.read(0x38, &mut buffer).is_ok() {
    //         rprintln!("Error in reading for touches");
    //     };
    //     for i in 0..16 {
    //         rprintln!("{}: {}", i, buffer[i] as u8);
    //     }
    // }

    // let ft5336 = Ft5336::new(i2c);

    // rprintln!("On 0x38 the registers contain:");
    // let mut byte: [u8; 10] = [0x01; 10];
    // for i in 0x00_u8..0x10 {
    //     if i2c.write(0x38, &byte).is_ok() {
    //         i2c.read(0x38, &mut byte);
    //         for i in 0..10 {
    //             rprintln!("Buffer {}: {}", i, byte[i] as u8);
    //         }
    //     } else {
    //         rprintln!("unsuccessful at writing");
    //     }
    // }

    // let style = MonoTextStyle::new(&PROFONT_24_POINT, Rgb565::YELLOW);

    // let a = match addresses {
    //     0 => "0",
    //     1 => "1",
    //     2 => "2",
    //     3 => "3",
    //     4 => "4",
    //     5 => "5",
    //     6 => "6",
    //     7 => "7",
    //     8 => "8",
    //     9 => "9",
    //     _ => "More",
    // };

    // loop {}
}
*/
