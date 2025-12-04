use embassy_sync::{channel::DynamicSender, watch::DynReceiver};

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
        todo!()
    }
}
