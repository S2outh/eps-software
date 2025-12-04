pub mod tmp100_drv;
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::{channel::DynamicSender, watch::DynReceiver};
use tmp100_drv::Tmp100;

use crate::EpsTelem;

const ERROR_TMP: i16 = i16::MIN;

pub struct Battery<'a, 'd> {
    temp_probe: Option<Tmp100<'a, I2c<'d, Async, Master>>>,
    adc_recv: DynReceiver<'a, i16>,
    tm_sender: DynamicSender<'a, EpsTelem>
}

impl<'a, 'd> Battery<'a, 'd> {
    pub async fn new(
        temp_probe: Option<Tmp100<'a, I2c<'d, Async, Master>>>,
        adc_recv: DynReceiver<'a, i16>,
        tm_sender: DynamicSender<'a, EpsTelem>,
    ) -> Self {
        Self {
            temp_probe,
            adc_recv,
            tm_sender
        }
    }
    async fn get_temperature(&mut self) -> Option<i16> {
        Some(self.temp_probe.as_mut()?.read_temp().await.ok()?)
    }
    async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
    pub async fn run(&mut self) {
        todo!()
    }
}
