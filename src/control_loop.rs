pub mod telecommands;

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::channel::{DynamicReceiver, DynamicSender};
use embassy_time::Timer;
use heapless::Vec;
use tmtc_definitions::telemetry;

use crate::EpsTelem;
use crate::pwr_src::d_flip_flop::{DFlipFlop, FlipFlopInput};
use crate::pwr_src::sink_ctrl::{Sink, SinkCtrl};
use telecommands::Telecommand;

const CONTROL_LOOP_TM_INTERVAL: u64 = 500;

pub struct ControlLoop<'d> {
    source_flip_flop: DFlipFlop<'d>,
    sink_ctrl: SinkCtrl<'d>,
    cmd_receiver: DynamicReceiver<'d, Telecommand>,
    tm_sender: DynamicSender<'d, EpsTelem>
}
impl<'d> ControlLoop<'d> {
    pub fn spawn(
        source_flip_flop: DFlipFlop<'d>,
        sink_ctrl: SinkCtrl<'d>,
        cmd_receiver: DynamicReceiver<'d, Telecommand>,
        tm_sender: DynamicSender<'d, EpsTelem>
    ) -> Self {
        Self {
            source_flip_flop,
            sink_ctrl,
            cmd_receiver,
            tm_sender
        }
    }

    async fn handle_cmd(&mut self, telecommand: Telecommand) {
        info!("cmd: {}", telecommand);
        match telecommand {
            Telecommand::SetSource(state, time) => {
                let old_state = self.source_flip_flop.get_state();
                self.source_flip_flop.set(state).await;
                if let Some(time) = time {
                    Timer::after_secs(time as u64).await;
                    self.source_flip_flop.set(old_state).await;
                }
            }
            Telecommand::EnableSink(sink, time) => {
                if self.sink_ctrl.is_enabled(sink) {
                    return;
                }
                self.sink_ctrl.enable(sink);
                if let Some(time) = time {
                    // this is blocking and prevents tm/tc during timeout.
                    // might be good to fix in the future
                    Timer::after_secs(time as u64).await;
                    self.sink_ctrl.disable(sink);
                }
            }
            Telecommand::DisableSink(sink, time) => {
                if !self.sink_ctrl.is_enabled(sink) {
                    return;
                }
                self.sink_ctrl.disable(sink);
                if let Some(time) = time {
                    // this is blocking and prevents tm/tc during timeout.
                    // might be good to fix in the future
                    Timer::after_secs(time as u64).await;
                    self.sink_ctrl.enable(sink);
                }
            }
        }
    }
    async fn send_state(&mut self) {
        let bitmap: u8 = self.source_flip_flop.is_enabled(FlipFlopInput::Bat1) as u8
            | (self.source_flip_flop.is_enabled(FlipFlopInput::AuxPwr) as u8) << 1
            | (self.sink_ctrl.is_enabled(Sink::Mainboard) as u8) << 2
            | (self.sink_ctrl.is_enabled(Sink::RocketLST) as u8) << 3
            | (self.sink_ctrl.is_enabled(Sink::RocketHD) as u8) << 4;

        let tm_data = Vec::from_array([bitmap]);
        self.tm_sender
            .send((telemetry::eps::EnableBitmap::ID, tm_data))
            .await;
    }

    pub async fn run(&mut self) {
        let event = select(
            self.cmd_receiver.receive(),
            Timer::after_millis(CONTROL_LOOP_TM_INTERVAL),
        ).await;
        match event {
            Either::First(cmd) => self.handle_cmd(cmd).await,
            Either::Second(()) => self.send_state().await,
        }
    }
}
