use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{DynamicReceiver, DynamicSender};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use south_common::definitions::telemetry::eps as tm;
use south_common::types::Telecommand;

use crate::EpsTMContainer;
use crate::pwr_src::d_flip_flop::DFlipFlop;
use crate::pwr_src::sink_ctrl::SinkCtrl;
use south_common::types::eps::{EPSCommand, FlipFlopInput, Sink, SinkEnabled, SourceEnabled};

const CTRL_LOOP_TM_INTERVAL: Duration = Duration::from_millis(500);
pub static SENSOR_CRITICAL: Signal<ThreadModeRawMutex, CriticalState> = Signal::new();

pub enum CriticalState {
    Temperature(FlipFlopInput),
    UnderVoltage(FlipFlopInput),
}

// control loop task
#[embassy_executor::task]
pub async fn ctrl_thread(mut control_loop: ControlLoop<'static>) -> ! {
    control_loop.run().await
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

    async fn handle_cmd(&mut self, cmd: Telecommand) {
        let Telecommand::EPS(telecommand) = cmd else {
            return;
        };
        match telecommand {
            EPSCommand::EnableSource(source, time) => {
                if self.source_flip_flop.is_enabled(source) {
                    return;
                }
                self.source_flip_flop.set(source).await;
                if let Some(time) = time {
                    // this is blocking and prevents tc during timeout.
                    // might be good to fix in the future
                    Timer::after_secs(time as u64).await;
                    self.source_flip_flop.reset(source).await;
                }
            }
            EPSCommand::DisableSource(source, time) => {
                if !self.source_flip_flop.is_enabled(source) {
                    return;
                }
                self.source_flip_flop.reset(source).await;
                if let Some(time) = time {
                    // this is blocking and prevents tc during timeout.
                    // might be good to fix in the future
                    Timer::after_secs(time as u64).await;
                    self.source_flip_flop.set(source).await;
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
            SinkEnabled::UMBILICAL,
            self.sink_ctrl.is_enabled(Sink::Umbilical),
        );
        sink_bitmap.set(
            SinkEnabled::ROCKET_LST_1,
            self.sink_ctrl.is_enabled(Sink::RocketLst1),
        );
        sink_bitmap.set(
            SinkEnabled::ROCKET_LST_2,
            self.sink_ctrl.is_enabled(Sink::RocketLst2),
        );
        sink_bitmap.set(
            SinkEnabled::SENSOR_LOWER,
            self.sink_ctrl.is_enabled(Sink::SensorLower),
        );
        sink_bitmap.set(
            SinkEnabled::ROCKET_HD,
            self.sink_ctrl.is_enabled(Sink::RocketHD),
        );
        sink_bitmap.set(
            SinkEnabled::BACKUP_SINK,
            self.sink_ctrl.is_enabled(Sink::BackupSink),
        );

        let container = EpsTMContainer::new(&tm::SinkEnabled, &sink_bitmap.bits()).unwrap();
        self.tm_sender.send(container).await;
    }
    async fn critical(&mut self, state: CriticalState) {
        match state {
            CriticalState::Temperature(source) => self.source_flip_flop.reset(source).await,
            CriticalState::UnderVoltage(source) => self.source_flip_flop.reset(source).await,
        }
    }
    pub async fn run(&mut self) -> ! {
        let mut tm_ticker = Ticker::every(CTRL_LOOP_TM_INTERVAL);

        loop {
            match select3(
                tm_ticker.next(),
                self.cmd_receiver.receive(),
                SENSOR_CRITICAL.wait(),
            ).await {
                Either3::First(_) => self.send_state().await,
                Either3::Second(cmd) => self.handle_cmd(cmd).await,
                Either3::Third(state) => self.critical(state).await,
            }
        }
    }
}
