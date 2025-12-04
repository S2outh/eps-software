use embassy_sync::{channel::DynamicSender, watch::DynReceiver};

use heapless::Vec;
use tmtc_definitions::{TMValue, telemetry};

use crate::EpsTelem;

pub struct AuxPwr<'a> {
    adc_recv: DynReceiver<'a, i16>,
    tm_sender: DynamicSender<'a, EpsTelem>
}

impl<'a> AuxPwr<'a> {
    pub async fn new(
        adc_recv: DynReceiver<'a, i16>,
        tm_sender: DynamicSender<'a, EpsTelem>
    ) -> Self {
        Self {
            adc_recv,
            tm_sender
        }
    }
    async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
    pub async fn run(&mut self) {
        let tm_data = Vec::from_array(self.get_voltage().await.to_bytes());
        self.tm_sender.send((telemetry::eps::AuxPowerVoltage::ID, tm_data)).await;
    }
}
