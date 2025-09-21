use embassy_stm32::{i2c::{Error, I2c, Master}, mode::Async};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

const TEMP_RANGE_TENTH_DEG: i32 = 128_0;

const TEMP_POINTER_REG: u8 = 0b00;
const CONFIG_POINTER_REG: u8 = 0b01;
const T_LOW_POINTER_REG: u8 = 0b10;
const T_HIGH_POINTER_REG: u8 = 0b11;

pub enum Resolution {
    BITS9,
    BITS10,
    BITS11,
    BITS12,
}
impl Resolution {
    pub fn get_temp_range(&self) -> i32 {
        match self {
            Self::BITS9 => 256,
            Self::BITS10 => 512,
            Self::BITS11 => 1024,
            Self::BITS12 => 2048,
        }
    }
    fn get_bit_shift(&self) -> u8 {
        match self {
            Self::BITS9 => 1,
            Self::BITS10 => 2,
            Self::BITS11 => 3,
            Self::BITS12 => 4,
        }
    }
    fn set_reg_bits(&self, config_reg: &mut u8) {
        *config_reg &= !(0b11 << 5);
        *config_reg |= (match self {
            Self::BITS9 => 0b00,
            Self::BITS10 => 0b01,
            Self::BITS11 => 0b10,
            Self::BITS12 => 0b11,
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

pub struct Tmp100<'a, 'd> {
    interface: &'a Mutex<ThreadModeRawMutex, I2c<'d, Async, Master>>,
    resolution: Resolution,
    addr_state: Addr0State
}

impl<'a, 'd> Tmp100<'a, 'd> {
    pub async fn new(
        interface: &'a Mutex<ThreadModeRawMutex, I2c<'d, Async, Master>>,
        resolution: Resolution,
        addr_state: Addr0State
    ) -> Result<Self, Error> {
        let mut config_reg = 0;
        resolution.set_reg_bits(&mut config_reg);
        interface.lock().await.write(addr_state.get_addr(), &[CONFIG_POINTER_REG, config_reg]).await?;
        interface.lock().await.write(addr_state.get_addr(), &[TEMP_POINTER_REG]).await?;
        Ok(Self { interface, resolution, addr_state })
    }
    pub fn raw_temp_range(&self) -> i32 {
        self.resolution.get_temp_range()
    }
    pub async fn read_temp_raw(&mut self) -> Result<i32, Error> {
        let mut buffer = [0u8; 2];
        self.interface.lock().await.read(self.addr_state.get_addr(), &mut buffer).await?;
        let bitshift = self.resolution.get_bit_shift();
        let tmp_raw = (buffer[0] as i32) << bitshift | (buffer[1] as i32) >> (8 - bitshift);
        Ok(tmp_raw)
    }
    pub async fn read_temp(&mut self) -> Result<i16, Error> {
        Ok(((self.read_temp_raw().await? * TEMP_RANGE_TENTH_DEG) / self.raw_temp_range()) as i16)
    }
}
