use embassy_stm32::{i2c::{I2c, Error}, mode::Async};
use embassy_sync::mutex::Mutex;


const TEMP_RANGE_TENTH_DEG: i32 = 128_0;

const TEMP_POINTER_REG: u8 = 0b00;
const CONFIG_POINTER_REG: u8 = 0b01;
const T_LOW_POINTER_REG: u8 = 0b10;
const T_HIGH_POINTER_REG: u8 = 0b11;

pub enum Resolution {
    Res9Bit,
    Res10Bit,
    Res11Bit,
    Res12Bit
}
impl Resolution {
    pub fn get_temp_range(&self) -> i32 {
        match self {
            Self::Res9Bit => 256,
            Self::Res10Bit => 512,
            Self::Res11Bit => 1024,
            Self::Res12Bit => 2048,
        }
    }
    fn get_bit_shift(&self) -> u8 {
        match self {
            Self::Res9Bit => 1,
            Self::Res10Bit => 2,
            Self::Res11Bit => 3,
            Self::Res12Bit => 4,
        }
    }
    fn set_reg_bits(&self, config_reg: &mut u8) {
        *config_reg &= !(0b11 << 5);
        *config_reg |= (match self {
            Self::Res9Bit => 0b00,
            Self::Res10Bit => 0b01,
            Self::Res11Bit => 0b10,
            Self::Res12Bit => 0b11,
        } << 5);
    }
}
pub enum Addr0State {
    Floating,
    High,
    Low
}
impl Addr0State {
    pub fn get_addr(&self) -> u8 {
        match self {
            Self::Floating => 0b1001001,
            Self::High => 0b1001010,
            Self::Low => 0b1001000,
        }
    }
}

pub struct Tmp100<'d> {
    interface: I2c<'d, Async>,
    resolution: Resolution,
    addr_state: Addr0State
}

impl<'d> Tmp100<'d> {
    pub async fn new(mut interface: I2c<'d, Async>, resolution: Resolution, addr_state: Addr0State) -> Result<Self, Error> {
        let mut config_reg = 0;
        resolution.set_reg_bits(&mut config_reg);
        interface.write(addr_state.get_addr(), &[CONFIG_POINTER_REG, config_reg]).await?;
        interface.write(addr_state.get_addr(), &[TEMP_POINTER_REG]).await?;
        Ok(Self { interface, resolution, addr_state })
    }
    pub fn raw_temp_range(&self) -> i32 {
        self.resolution.get_temp_range()
    }
    pub async fn read_temp_raw(&mut self) -> Result<i32, Error> {
        let mut buffer = [0u8; 2];
        self.interface.read(self.addr_state.get_addr(), &mut buffer).await.unwrap();
        let bitshift = self.resolution.get_bit_shift();
        let tmp_raw = (buffer[0] as i32) << bitshift | (buffer[1] as i32) >> (8 - bitshift);
        Ok(tmp_raw)
    }
    pub async fn read_temp(&mut self) -> Result<i32, Error> {
        Ok((self.read_temp_raw().await? * TEMP_RANGE_TENTH_DEG) / self.raw_temp_range())
    }
}
