use core::fmt::Debug;

use bitflags::bitflags;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::Duration;
use embedded_hal::i2c::ErrorType;
use embedded_hal_async::i2c::I2c;

macro_rules! encode_reg16 {
    ($base:expr; $val:expr => $from:literal, $to:literal) => {
        let width: u16 = $to - $from + 1;
        let mask: u16 = ((1u32 << width) - 1) as u16;
        let shifted_mask: u16 = mask << $from;
        $base = ($base & !shifted_mask) | (($val as u16 & mask) << $from);
    };
    ($base:expr; $val:expr => $from:literal) => {
        encode_reg16!($base; $val => $from, $from)
    }
}

macro_rules! decode_reg16 {
    ($val:expr => $from:literal, $to:literal) => {{
        let width: u16 = $to - $from + 1;
        let mask: u16 = ((1u32 << width) - 1) as u16;
        ($val >> $from) & mask
    }};
    ($val:expr => $from:literal) => {
        decode_reg16!($val => $from, $from)
    }
}

#[derive(defmt::Format)]
pub enum Error<I2CError> {
    IO(I2CError),
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
impl AvgMode {
    fn samples(&self) -> u32 {
        match self {
            Self::Sample1     => 1,
            Self::Sample4     => 4,
            Self::Sample16    => 16,
            Self::Sample64    => 64,
            Self::Sample128   => 128,
            Self::Sample256   => 256,
            Self::Sample512   => 512,
            Self::Sample1024  => 1024,
        }
    }
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

impl ConversionTime {
    fn duration(&self) -> Duration {
        Duration::from_micros(match self {
            Self::Micros140   => 140,
            Self::Micros204   => 204,
            Self::Micros332   => 332,
            Self::Micros588   => 588,
            Self::Micros1100  => 1100,
            Self::Micros2116  => 2116,
            Self::Micros4156  => 4156,
            Self::Micros8244  => 8244,
        })
    }
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
impl OperatingMode {
    fn bus_active(&self) -> bool {
        matches!(self, Self::BusSingleShot) ||
        matches!(self, Self::BusContinuous) ||
        matches!(self, Self::AllSingleShot) ||
        matches!(self, Self::AllContinuous)
    }
    fn shunt_active(&self) -> bool {
        matches!(self, Self::ShuntSingleShot) ||
        matches!(self, Self::ShuntContinuous) ||
        matches!(self, Self::AllSingleShot) ||
        matches!(self, Self::AllContinuous)
    }
}

pub struct InaConfig {
    reset: bool,
    pub channels: Channel,
    pub avg_mode: AvgMode,
    pub bus_conversion_time: ConversionTime,
    pub shunt_conversion_time: ConversionTime,
    pub mode: OperatingMode,
}
impl InaConfig {
    pub fn new() -> Self {
        // setup default options from the datasheet
        Self { 
            reset: false,
            channels: Channel::all(),
            avg_mode: AvgMode::Sample1,
            bus_conversion_time: ConversionTime::Micros1100,
            shunt_conversion_time: ConversionTime::Micros1100,
            mode: OperatingMode::AllContinuous,
        }
    }
    pub fn reset() -> Self {
        Self {
            reset: true,
            ..Self::new()
        }
    }
    pub fn calculate_cycle_time(&self) -> Duration {
        let num_active_channels = self.channels.iter().count();

        let bus_conversion_duration = if self.mode.bus_active() {
            self.bus_conversion_time.duration()
        } else {
            Duration::MIN
        };

        let shunt_conversion_duration = if self.mode.shunt_active() {
            self.shunt_conversion_time.duration()
        } else {
            Duration::MIN
        };

        num_active_channels as u32
            * self.avg_mode.samples()
            * (bus_conversion_duration + shunt_conversion_duration)
    }
    fn build(self) -> u16 {
        let mut conf = 0;
        encode_reg16!(conf; self.reset as u8 => 15);
        encode_reg16!(conf; self.channels.bits() => 12, 14);
        encode_reg16!(conf; self.avg_mode as u8 => 9, 11);
        encode_reg16!(conf; self.bus_conversion_time as u8 => 6, 8);
        encode_reg16!(conf; self.shunt_conversion_time as u8 => 3, 5);
        encode_reg16!(conf; self.mode as u8 => 0, 2);
        conf
    }
}

pub struct Ina<'d, I2C: I2c + ErrorType> {
    i2c: &'d Mutex<ThreadModeRawMutex, I2C>,
    i2c_addr: A0,
}

impl<'d, I2C: I2c + ErrorType> Ina<'d, I2C> {
    pub async fn new(
        i2c: &'d Mutex<ThreadModeRawMutex, I2C>,
        i2c_addr: A0,
    ) -> Result<Self, Error<I2C::Error>> {
        let mut ina = Self {
            i2c,
            i2c_addr,
        };
        ina.reset().await?;
        Ok(ina)
    }

    async fn write_reg(&mut self, reg_addr: &dyn Register, data: u16) -> Result<(), Error<I2C::Error>> {
        let data = data.to_be_bytes();
        let buf = [reg_addr.get_addr(), data[0], data[1]];
        self.i2c.lock().await
            .write(self.i2c_addr as u8, &buf)
            .await
            .map_err(Error::IO)
    }

    async fn read_reg(&mut self, reg_addr: &dyn Register) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c.lock().await
            .write_read(self.i2c_addr as u8, core::array::from_ref(&reg_addr.get_addr()), &mut buf)
            .await
            .map_err(Error::IO)?;

        Ok(u16::from_be_bytes(buf))
    }

    // configuration
    pub async fn write_conf(&mut self, config: InaConfig) -> Result<(), Error<I2C::Error>> {
        self.write_reg(&ConfigRegisters::Configuration, config.build()).await
    }
    pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_conf(InaConfig::reset()).await
    }
    pub async fn set_cw_limit(&mut self, register: CWLimitRegisters, voltage_uv: u32) -> Result<(), Error<I2C::Error>> {
        let value = (voltage_uv / 40) as u16; // LSB is 40 uV
        self.write_reg(&register, value << 3).await
    }
    pub async fn set_pv_limit(&mut self, register: PVLimitRegisters, voltage_mv: i32) -> Result<(), Error<I2C::Error>> {
        let value = voltage_mv / 8; // LSB is 8 mV
        self.write_reg(&register, (value << 3) as u16).await
    }
    pub async fn set_all_critical_limits(&mut self, voltage_uv: u32) -> Result<(), Error<I2C::Error>> {
        self.set_cw_limit(CWLimitRegisters::Channel1CriticalLim, voltage_uv).await?;
        self.set_cw_limit(CWLimitRegisters::Channel2CriticalLim, voltage_uv).await?;
        self.set_cw_limit(CWLimitRegisters::Channel3CriticalLim, voltage_uv).await?;
        Ok(())
    }
    pub async fn set_all_warning_limits(&mut self, voltage_uv: u32) -> Result<(), Error<I2C::Error>> {
        self.set_cw_limit(CWLimitRegisters::Channel1WarningLim, voltage_uv).await?;
        self.set_cw_limit(CWLimitRegisters::Channel2WarningLim, voltage_uv).await?;
        self.set_cw_limit(CWLimitRegisters::Channel3WarningLim, voltage_uv).await?;
        Ok(())
    }

    // read data
    pub async fn read_voltage_reg(&mut self, register: VoltageRegisters) -> Result<i16, Error<I2C::Error>> {
        let data = self.read_reg(&register).await?;
        Ok(data as i16 >> 3)
    }
    async fn read_state(&mut self) -> Result<(Channel, Channel), Error<I2C::Error>> {
        let mask_reg = self.read_reg(&ConfigRegisters::MaskEnable).await?;
        let warning_states = Channel::from_bits_truncate(decode_reg16!(mask_reg => 7, 9) as u8);
        let critical_states = Channel::from_bits_truncate(decode_reg16!(mask_reg => 3, 5) as u8);

        Ok((warning_states, critical_states))
    }
}
