use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use south_common::definitions::telemetry::eps as tm;

use crate::pwr_src::d_flip_flop::DFlipFlop;
use crate::pwr_src::sink_ctrl::SinkCtrl;
use crate::{EpsChellUnion, EpsTCReceiver, EpsTMSender};
use south_common::types::eps::{EPSCommand, FlipFlopInput, Sink, SinkEnabled, SourceEnabled};

pub static SENSOR_CRITICAL: Signal<ThreadModeRawMutex, CriticalState> = Signal::new();

const REBOOT_TIME: Duration = Duration::from_millis(100);

pub enum CriticalState {
    Temperature(FlipFlopInput),
    UnderVoltage(FlipFlopInput),
}

pub struct ControlLoop<'d> {
    source_flip_flop: DFlipFlop<'d>,
    sink_ctrl: SinkCtrl<'d>,
    cmd_receiver: EpsTCReceiver,
    tm_sender: EpsTMSender,
}

impl<'d> ControlLoop<'d> {
    pub fn spawn(
        source_flip_flop: DFlipFlop<'d>,
        sink_ctrl: SinkCtrl<'d>,
        cmd_receiver: EpsTCReceiver,
        tm_sender: EpsTMSender,
    ) -> Self {
        Self {
            source_flip_flop,
            sink_ctrl,
            cmd_receiver,
            tm_sender,
        }
    }

    async fn handle_cmd(&mut self, telecommand: EPSCommand) {
        match telecommand {
            EPSCommand::EnableSource(source) => {
                self.source_flip_flop.set(source).await;
            }
            EPSCommand::DisableSource(source) => {
                self.source_flip_flop.reset(source).await;
            }
            EPSCommand::RebootSource(source) => {
                self.source_flip_flop.reset(source).await;
                Timer::after(REBOOT_TIME).await;
                self.source_flip_flop.set(source).await;
            }
            EPSCommand::EnableSink(sink) => {
                self.sink_ctrl.enable(sink);
            }
            EPSCommand::DisableSink(sink) => {
                self.sink_ctrl.disable(sink);
            }
            EPSCommand::RebootSink(sink) => {
                self.sink_ctrl.disable(sink);
                Timer::after(REBOOT_TIME).await;
                self.sink_ctrl.enable(sink);
            }
        }
    }
    async fn send_state(&mut self) {
        macro_rules! send_state_bitmap {
            ($state: path, $source_ident: ident, $enum: path, $($field:ident),*) => { paste::paste! {
                let mut bitmap = $state::empty();
                $(
                    bitmap.set(
                        $state::$field,
                        self.$source_ident.is_enabled($enum::[<$field:camel>])
                    );
                )*
                let container = EpsChellUnion::new(&tm:: $state, &bitmap.bits()).unwrap();
                self.tm_sender.send(container).await;
            } };
        }
        send_state_bitmap!(
            SourceEnabled,
            source_flip_flop,
            FlipFlopInput,
            BAT_1,
            BAT_2,
            AUX_PWR
        );
        send_state_bitmap!(
            SinkEnabled,
            sink_ctrl,
            Sink,
            CARRIER,
            UMBILICAL,
            ROCKET_LST_1,
            ROCKET_LST_2,
            SENSOR_LOWER,
            ROCKET_H_D,
            BACKUP_SINK
        );
    }
    async fn critical(&mut self, state: CriticalState) {
        match state {
            CriticalState::Temperature(source) => self.source_flip_flop.reset(source).await,
            CriticalState::UnderVoltage(source) => self.source_flip_flop.reset(source).await,
        }
    }
    pub async fn run(&mut self) -> ! {
        const CTRL_LOOP_TM_INTERVAL: Duration = Duration::from_millis(500);
        let mut tm_ticker = Ticker::every(CTRL_LOOP_TM_INTERVAL);

        loop {
            match select3(
                tm_ticker.next(),
                self.cmd_receiver.receive(),
                SENSOR_CRITICAL.wait(),
            )
            .await
            {
                Either3::First(_) => self.send_state().await,
                Either3::Second(cmd) => self.handle_cmd(cmd).await,
                Either3::Third(state) => self.critical(state).await,
            }
        }
    }
}
