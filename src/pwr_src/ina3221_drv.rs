use bitflags::bitflags;
use embassy_stm32::{
    exti::ExtiInput, i2c::{I2c, mode::Master}, mode::Async,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

type I2cError = embassy_stm32::i2c::Error;

#[macro_export]
macro_rules! encode_reg16 {
    ($base:expr; $val:expr => $shift:literal, $width:literal) => {
        {
            let mask: u16 = ((1u32 << $width) - 1) as u16;
            let shifted_mask: u16 = mask << $shift;
            $base = ($base & !shifted_mask) | (($val as u16 & mask) << $shift);
        }
    };
}

#[derive(defmt::Format)]
pub enum Error {
    IO(I2cError),
}

// I2c address
#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum A0 {
    Ground = 0b1000000,
    VS     = 0b1000001,
    SDA    = 0b1000010,
    SCL    = 0b1000011,
}

// Registers
trait Register {
    fn get_addr(&self) -> u8;
}
macro_rules! derive_reg {
    ($reg_type:ident) => {
        impl Register for $reg_type {
            fn get_addr(&self) -> u8 {
                *self as u8
            }
        }
    };
}
#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
enum ConfigRegisters {
    Configuration       = 0x00,
    MaskEnable          = 0x0F,
}
#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum VoltageRegisters {
    Channel1Shunt       = 0x01,
    Channel1Bus         = 0x02,
    Channel2Shunt       = 0x03,
    Channel2Bus         = 0x04,
    Channel3Shunt       = 0x05,
    Channel3Bus         = 0x06,
}
#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum CWLimitRegisters {
    Channel1CriticalLim = 0x07,
    Channel1WarningLim  = 0x08,
    Channel2CriticalLim = 0x09,
    Channel2WarningLim  = 0x0A,
    Channel3CriticalLim = 0x0B,
    Channel3WarningLim  = 0x0C,
}
#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum PVLimitRegisters {
    PowerValidUpperLim  = 0x10,
    PowerValidLowerLim  = 0x11,
}
#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum ShuntVoltageSumRegisters {
    ShuntVoltageSum     = 0x0D,
    ShuntVoltageSumLim  = 0x0E,
}
derive_reg!(ConfigRegisters);
derive_reg!(VoltageRegisters);
derive_reg!(CWLimitRegisters);
derive_reg!(PVLimitRegisters);
derive_reg!(ShuntVoltageSumRegisters);

// Config reg fields
bitflags! {
    pub struct Channel: u8 {
        const Ch1 = 1 << 2;
        const Ch2 = 1 << 1;
        const Ch3 = 1 << 0;
    }
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum AvgMode {
    Sample1     = 0,
    Sample4     = 1,
    Sample16    = 2,
    Sample64    = 3,
    Sample128   = 4,
    Sample256   = 5,
    Sample512   = 6,
    Sample1024  = 7,
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum ConversionTime {
    Micros140   = 0,
    Micros204   = 1,
    Micros332   = 2,
    Micros588   = 3,
    Micros1100  = 4,
    Micros2116  = 5,
    Micros4156  = 6,
    Micros8244  = 7,
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum OperatingMode {
    PowerDown       = 0,
    ShuntSingleShot = 1,
    BusSingleShot   = 2,
    AllSingleShot   = 3,
    // PowerDown 2  = 4,
    ShuntContinuous = 5,
    BusContinuous   = 6,
    AllContinuous   = 7,
}

pub struct InaConfig {
    conf: u16
}
impl InaConfig {
    pub fn new() -> Self {
        Self { conf: 0 }
    }
    pub fn reset(mut self) -> Self {
        encode_reg16!(self.conf; 1 => 15, 1);
        self
    }
    pub fn channels(mut self, ch: Channel) -> Self {
        encode_reg16!(self.conf; ch.bits() => 12, 3);
        self
    }
    pub fn average(mut self, avg: AvgMode) -> Self {
        encode_reg16!(self.conf; avg as u8 => 9, 3);
        self
    }
    pub fn bus_conversion(mut self, time: ConversionTime) -> Self {
        encode_reg16!(self.conf; time as u8 => 6, 3);
        self
    }
    pub fn shunt_conversion(mut self, time: ConversionTime) -> Self {
        encode_reg16!(self.conf; time as u8 => 3, 3);
        self
    }
    pub fn mode(mut self, mode: OperatingMode) -> Self {
        encode_reg16!(self.conf; mode as u8 => 0, 3);
        self
    }
    pub fn build(self) -> u16 {
        self.conf
    }
}

pub struct Ina<'d> {
    i2c: &'d Mutex<ThreadModeRawMutex, I2c<'d, Async, Master>>,
    i2c_addr: A0,
    int: ExtiInput<'d>,
}

impl<'d> Ina<'d> {
    pub async fn new(
        i2c: &'d Mutex<ThreadModeRawMutex, I2c<'d, Async, Master>>,
        i2c_addr: A0,
        int: ExtiInput<'d>,
    ) -> Result<Self, Error> {
        let mut ina = Self {
            i2c,
            i2c_addr,
            int,
        };
        ina.reset().await?;
        Ok(ina)
    }

    async fn write_reg(&mut self, reg_addr: &dyn Register, data: u16) -> Result<(), Error> {
        let data = data.to_be_bytes();
        let buf = [reg_addr.get_addr(), data[0], data[1]];
        self.i2c.lock().await
            .write(self.i2c_addr as u8, &buf)
            .await
            .map_err(Error::IO)
    }

    async fn read_reg(&mut self, reg_addr: &dyn Register) -> Result<u16, Error> {
        let mut buf = [0u8; 2];
        self.i2c.lock().await
            .write_read(self.i2c_addr as u8, core::array::from_ref(&reg_addr.get_addr()), &mut buf)
            .await
            .map_err(Error::IO)?;

        Ok(u16::from_be_bytes(buf))
    }

    pub async fn write_conf(&mut self, config: InaConfig) -> Result<(), Error> {
        self.write_reg(&ConfigRegisters::Configuration, config.build()).await
    }
    pub async fn reset(&mut self) -> Result<(), Error> {
        self.write_conf(InaConfig::new().reset()).await
    }
    // async fn read_masks(&mut self) -> Result<(), Error> {
    //     let mask_reg = self.read_reg(&ConfigRegisters::MaskEnable).await?;
    //     
    // }
    pub async fn read_voltage_data(&mut self, register: VoltageRegisters) -> Result<i16, Error> {
        let data = self.read_reg(&register).await?;
        Ok(data as i16 >> 3)
    }
    pub async fn set_cw_limit(&mut self, register: CWLimitRegisters, value: u16) -> Result<(), Error> {
        self.write_reg(&register, value << 3).await
    }
    pub async fn set_pv_limit(&mut self, register: PVLimitRegisters, value: i16) -> Result<(), Error> {
        self.write_reg(&register, (value << 3) as u16).await
    }
}
