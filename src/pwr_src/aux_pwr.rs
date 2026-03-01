use embassy_sync::{channel::DynamicSender};

use embassy_time::{Duration, Ticker};
//use south_common::definitions::telemetry::eps as tm;

use crate::EpsTMContainer;

// Aux pwr task
#[embassy_executor::task]
pub async fn aux_pwr_thread(mut aux_pwr: AuxPwr<'static>) {
    const AUX_LOOP_LEN: Duration = Duration::from_millis(500);
    let mut ticker = Ticker::every(AUX_LOOP_LEN);
    loop {
        aux_pwr.run().await;
        ticker.next().await;
    }
}

pub struct AuxPwr<'a> {
    tm_sender: DynamicSender<'a, EpsTMContainer>,
}

impl<'a> AuxPwr<'a> {
    pub async fn new(
        tm_sender: DynamicSender<'a, EpsTMContainer>,
    ) -> Self {
        Self {
            tm_sender,
        }
    }
    pub async fn run(&mut self) {
    }
}
