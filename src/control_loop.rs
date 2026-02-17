use embassy_futures::select::{Either, select};
use embassy_sync::channel::{DynamicReceiver, DynamicSender};
use embassy_time::{Duration, Instant, Timer};
use south_common::definitions::telemetry::eps as tm;

use crate::EpsTMContainer;
use crate::pwr_src::d_flip_flop::{DFlipFlop, FlipFlopInput};
use crate::pwr_src::sink_ctrl::SinkCtrl;
use south_common::types::{EPSCommand, EPSEnabled, Sink, Telecommand};

const CTRL_LOOP_TM_INTERVAL: Duration = Duration::from_millis(500);

// control loop task
#[embassy_executor::task]
pub async fn ctrl_thread(mut control_loop: ControlLoop<'static>) {
    loop {
        control_loop.run().await;
    }
}

pub struct ControlLoop<'d> {
    source_flip_flop: DFlipFlop<'d>,
    sink_ctrl: SinkCtrl<'d>,
    next_tm: Instant,
    cmd_receiver: DynamicReceiver<'d, Telecommand>,
    tm_sender: DynamicSender<'d, EpsTMContainer>,
}

impl<'d> ControlLoop<'d> {
    pub fn spawn(
        source_flip_flop: DFlipFlop<'d>,
        sink_ctrl: SinkCtrl<'d>,
        cmd_receiver: DynamicReceiver<'d, Telecommand>,
        tm_sender: DynamicSender<'d, EpsTMContainer>,
    ) -> Self {
        Self {
            source_flip_flop,
            sink_ctrl,
            next_tm: Instant::now(),
            cmd_receiver,
            tm_sender,
        }
    }

    async fn handle_cmd(&mut self) {
        let Telecommand::EPS(telecommand) = self.cmd_receiver.receive().await else {
            return;
        };
        match telecommand {
            EPSCommand::SetSource(state, time) => {
                let old_state = self.source_flip_flop.get_state();
                self.source_flip_flop.set(state).await;
                if let Some(time) = time {
                    // this is blocking and prevents tc during timeout.
                    // might be good to fix in the future
                    Timer::after_secs(time as u64).await;
                    self.source_flip_flop.set(old_state).await;
                }
            }
            EPSCommand::EnableSink(sink, time) => {
                if self.sink_ctrl.is_enabled(sink) {
                    return;
                }
                self.sink_ctrl.enable(sink);
                if let Some(time) = time {
                    // this is blocking and prevents tc during timeout.
                    // might be good to fix in the future
                    Timer::after_secs(time as u64).await;
                    self.sink_ctrl.disable(sink);
                }
            }
            EPSCommand::DisableSink(sink, time) => {
                if !self.sink_ctrl.is_enabled(sink) {
                    return;
                }
                self.sink_ctrl.disable(sink);
                if let Some(time) = time {
                    // this is blocking and prevents tc during timeout.
                    // might be good to fix in the future
                    Timer::after_secs(time as u64).await;
                    self.sink_ctrl.enable(sink);
                }
            }
        }
    }
    async fn send_state(&mut self) {
        Timer::at(self.next_tm).await;
        
        let mut bitmap = EPSEnabled::empty();
        bitmap.set(
            EPSEnabled::BAT1,
            self.source_flip_flop.is_enabled(FlipFlopInput::Bat1),
        );
        bitmap.set(
            EPSEnabled::BAT2,
            self.source_flip_flop.is_enabled(FlipFlopInput::Bat2),
        );
        bitmap.set(
            EPSEnabled::AUXPWR,
            self.source_flip_flop.is_enabled(FlipFlopInput::AuxPwr),
        );
        bitmap.set(
            EPSEnabled::ROCKETLST,
            self.sink_ctrl.is_enabled(Sink::RocketLST),
        );
        bitmap.set(
            EPSEnabled::SENSORUPP,
            self.sink_ctrl.is_enabled(Sink::SensorUpper),
        );
        bitmap.set(EPSEnabled::ROCKETHD, self.sink_ctrl.is_enabled(Sink::RocketHD));

        let container = EpsTMContainer::new(&tm::EnableBitmap, &bitmap.bits()).unwrap();
        self.tm_sender.send(container).await;
    }
    pub async fn run(&mut self) {
        if let Either::First(_) = select(
            Timer::at(self.next_tm),
            self.handle_cmd()
        ).await {
            self.send_state().await;
            self.next_tm += CTRL_LOOP_TM_INTERVAL;
        }
    }
}
