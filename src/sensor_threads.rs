use embassy_stm32::{i2c::Master, mode::Async};
use embassy_sync::channel::DynamicSender;
use embassy_time::{Duration, Ticker};
use south_common::{definitions::telemetry::eps as tm, tmtc_system::TelemetryDefinition, types::eps::FlipFlopInput};
use defmt::{expect, info};

use crate::{EpsTMContainer, control_loop::{self, CriticalState}, pwr_src::{ina3221_drv::{Ina, InaConfig, VoltageRegisters}, tmp100_drv::Tmp100}};

type I2c = embassy_stm32::i2c::I2c<'static, Async, Master>;

#[embassy_executor::task]
pub async fn ina_thread(
    tm_sender: DynamicSender<'static, EpsTMContainer>,
    mut ina: Ina<'static, I2c>
) {
    macro_rules! send_voltage {
        ($channel:expr, $tm_def:path $(, $additional_fn:expr)?) => {
            if let Ok(value) = ina.read_voltage_reg($channel).await {
                $($additional_fn(value);)?
                let container = EpsTMContainer::new(&$tm_def, &value).unwrap();
                tm_sender.send(container).await;
            }
        };
    }

    fn check_under_voltage(v: i16, input: FlipFlopInput) {
        const CRITICAL_VOLTAGE_V: f32 = 15.;
        let voltage = (v as f32 * 8.) / 1000.;
        if voltage <= CRITICAL_VOLTAGE_V {
            control_loop::SENSOR_CRITICAL.signal(CriticalState::UnderVoltage(input))
        }
    }

    let mut ina_config = InaConfig::new();
    ina_config.avg_mode = crate::pwr_src::ina3221_drv::AvgMode::Sample256;
    ina_config.bus_conversion_time = crate::pwr_src::ina3221_drv::ConversionTime::Micros4156;
    ina_config.shunt_conversion_time = crate::pwr_src::ina3221_drv::ConversionTime::Micros1100;

    let cycle_duration = ina_config.calculate_cycle_time();
    let mut ticker = Ticker::every(cycle_duration);
    info!("Calculated ina cycle duration: {}ms", cycle_duration.as_millis());

    expect!(ina.write_conf(ina_config).await, "could not write ina config");

    let bat_1_under_voltage_check = |v| check_under_voltage(v, FlipFlopInput::Bat1);
    let bat_2_under_voltage_check = |v| check_under_voltage(v, FlipFlopInput::Bat2);

    loop {
        ticker.next().await;

        send_voltage!(VoltageRegisters::Channel3Bus, tm::AuxPowerVoltage);
        send_voltage!(VoltageRegisters::Channel2Bus, tm::Bat1Voltage, bat_1_under_voltage_check);
        send_voltage!(VoltageRegisters::Channel1Bus, tm::Bat2Voltage, bat_2_under_voltage_check);

        send_voltage!(VoltageRegisters::Channel3Shunt, tm::AuxPowerCurrent);
        send_voltage!(VoltageRegisters::Channel2Shunt, tm::Bat1Current);
        send_voltage!(VoltageRegisters::Channel1Shunt, tm::Bat2Current);
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn bat_temp_thread(
    tm_sender: DynamicSender<'static, EpsTMContainer>,
    mut tmp: Tmp100<'static, I2c>,
    topic: &'static dyn TelemetryDefinition,
    source: FlipFlopInput,
) {
    const TEMP_LOOP_LEN: Duration = Duration::from_secs(2);
    const CRITICAL_TEMPERATURE_C: i16 = 90;
    let mut ticker = Ticker::every(TEMP_LOOP_LEN);
    loop {
        if let Ok(temperature) = tmp.read_temp().await {
            if temperature / 10 >= CRITICAL_TEMPERATURE_C {
                control_loop::SENSOR_CRITICAL.signal(CriticalState::Temperature(source));
            }
            let container = EpsTMContainer::new(topic, &temperature).unwrap();
            tm_sender.send(container).await;
        }
        ticker.next().await;
    }
}
