use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use rtt_target::rprintln;
use stm32f7xx_hal::{
    i2c::{BlockingI2c, Mode},
    pac::I2C3,
    prelude::*,
    rcc::{Clocks, APB1},
};

const FT5336_TOUCHPAD_ADDR: u8 = 0x38;

pub struct Touch<I2C3, SCL, SDA> {
    i2c: BlockingI2c<I2C3, SCL, SDA>,
    addr: u8,
}
impl<
        SCL: stm32f7xx_hal::i2c::PinScl<stm32f7xx_hal::pac::I2C3>,
        SDA: stm32f7xx_hal::i2c::PinSda<stm32f7xx_hal::pac::I2C3>,
    > Touch<I2C3, SCL, SDA>
{
    pub fn new(
        i2c_port: I2C3,
        scl: SCL,
        sda: SDA,
        apb1: &mut APB1,
        clocks: Clocks,
    ) -> Touch<I2C3, SCL, SDA> {
        rprintln!("Initialising I2C");
        let mode = Mode::fast(100_000_u32.Hz());

        let i2c = BlockingI2c::i2c3(i2c_port, (scl, sda), mode, clocks, apb1, 10_000);

        Touch {
            i2c,
            addr: FT5336_TOUCHPAD_ADDR,
        }
    }

    pub fn test(mut self) {
        rprintln!("Testing I2C");

        let cmd: [u8; 1] = [0];
        if self.i2c.write(self.addr, &cmd).is_ok() {
            rprintln!("Wrote to touchpad okay");
        }
    }
}
