use embassy_futures::select::{Either, select};
use embassy_sync::channel::{DynamicReceiver, DynamicSender};
use embassy_time::{Duration, Instant, Timer};
use south_common::definitions::telemetry::eps as tm;

use crate::EpsTMContainer;
use crate::pwr_src::d_flip_flop::{DFlipFlop, FlipFlopInput};
use crate::pwr_src::sink_ctrl::SinkCtrl;
use south_common::types::{EPSCommand, SinkEnabled, SourceEnabled, Sink, Telecommand};

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

// macro_rules! populate_bitmap {
//     ($bm:ident, $(($field:ident, $fn:ident)),*) => {
//         paste::paste!{
//             let [<$bm:snake>] = $bm::empty();
//             $(
//                 [<$bm:snake>].set(
//                     $bm::[<$field:snake:upper>],
//                     $fn
//                 );
//             )*
//         }
//     };
// }

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
        
        let mut source_bitmap = SourceEnabled::empty();
        source_bitmap.set(
            SourceEnabled::BAT_1,
            self.source_flip_flop.is_enabled(FlipFlopInput::Bat1),
        );
        source_bitmap.set(
            SourceEnabled::BAT_2,
            self.source_flip_flop.is_enabled(FlipFlopInput::Bat2),
        );
        source_bitmap.set(
            SourceEnabled::AUX_PWR,
            self.source_flip_flop.is_enabled(FlipFlopInput::AuxPwr),
        );

        let container = EpsTMContainer::new(&tm::SourceEnabled, &source_bitmap.bits()).unwrap();
        self.tm_sender.send(container).await;

        let mut sink_bitmap = SinkEnabled::empty();
        sink_bitmap.set(
            SinkEnabled::CARRIER,
            self.sink_ctrl.is_enabled(Sink::Carrier),
        );
        sink_bitmap.set(
            SinkEnabled::ROCKETLST,
            self.sink_ctrl.is_enabled(Sink::RocketLST),
        );
        sink_bitmap.set(
            SinkEnabled::GPS,
            self.sink_ctrl.is_enabled(Sink::GPS),
        );
        sink_bitmap.set(
            SinkEnabled::EXT_CAM,
            self.sink_ctrl.is_enabled(Sink::ExternalCamera),
        );
        sink_bitmap.set(
            SinkEnabled::LOWER_SENS,
            self.sink_ctrl.is_enabled(Sink::SensorLower),
        );
        sink_bitmap.set(
            SinkEnabled::ROCKETHD,
            self.sink_ctrl.is_enabled(Sink::RocketHD),
        );
        sink_bitmap.set(
            SinkEnabled::BACKUP,
            self.sink_ctrl.is_enabled(Sink::BackupSink),
        );

        let container = EpsTMContainer::new(&tm::SinkEnabled, &sink_bitmap.bits()).unwrap();
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
