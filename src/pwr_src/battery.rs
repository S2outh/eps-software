pub mod tmp100_drv;
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::{channel::DynamicSender, watch::DynReceiver};
use embassy_time::Timer;
use tmp100_drv::Tmp100;

use south_common::TelemetryDefinition;

use crate::EpsTMContainer;

// Battery task
#[embassy_executor::task(pool_size = 2)]
pub async fn battery_thread(mut battery: Battery<'static, 'static>) {
    const BTRY_LOOP_LEN_MS: u64 = 500;
    loop {
        battery.run().await;
        Timer::after_millis(BTRY_LOOP_LEN_MS).await;
    }
}

pub struct Battery<'a, 'd> {
    temp_probe: Option<Tmp100<'a, I2c<'d, Async, Master>>>,
    adc_recv: DynReceiver<'a, i16>,
    tm_sender: DynamicSender<'a, EpsTMContainer>,
    temp_topic: &'static dyn TelemetryDefinition,
    voltage_topic: &'static dyn TelemetryDefinition,
}

impl<'a, 'd> Battery<'a, 'd> {
    pub async fn new(
        temp_probe: Option<Tmp100<'a, I2c<'d, Async, Master>>>,
        adc_recv: DynReceiver<'a, i16>,
        tm_sender: DynamicSender<'a, EpsTMContainer>,
        temp_topic: &'static dyn TelemetryDefinition,
        voltage_topic: &'static dyn TelemetryDefinition,
    ) -> Self {
        Self {
            temp_probe,
            adc_recv,
            tm_sender,
            temp_topic,
            voltage_topic,
        }
    }
    async fn get_temperature(&mut self) -> Option<i16> {
        Some(self.temp_probe.as_mut()?.read_temp().await.ok()?)
    }
    async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
    pub async fn run(&mut self) {
        if let Some(temperature) = self.get_temperature().await {
            let container = EpsTMContainer::new(self.temp_topic, &temperature).unwrap();
            self.tm_sender.send(container).await;
        }

        let container = EpsTMContainer::new(self.voltage_topic, &self.get_voltage().await).unwrap();
        self.tm_sender.send(container).await;
    }
}
