mod telecommands;

use defmt::{error, info};
use embassy_futures::select::{select, Either};
use embassy_stm32::can::{CanConfigurator, RxBuf, TxBuf};
use embassy_sync::watch::DynReceiver;
use embassy_time::Timer;
use rodos_can_interface::{ActivePeriph, RodosCanInterface};
use static_cell::StaticCell;

use crate::control_loop::telecommands::Telecommand;
use crate::pwr_src::d_flip_flop::{DFlipFlop, FlipFlopInput};
use crate::pwr_src::sink_ctrl::{Sink, SinkCtrl};

use super::pwr_src::battery::Battery;
use super::pwr_src::aux_pwr::AuxPwr;

const RODOS_DEVICE_ID: u8 = 0x02;

#[repr(u16)]
enum TopicId {
    Cmd = 1100,
    TelemReq = 1102,
    TelemEnableBM = 1410,
    TelemBat1Tmp = 1411,
    TelemInternalTmp = 1412,
    TelemBat1Voltage = 1413,
    TelemAuxPwrVoltage = 1414,
}

const ACTIVATION_VOLTAGE_THRESHHOLD_10X_MV: i16 = 15_00;
const ERROR_TMP: i16 = i16::MIN;

const RODOS_MAX_RAW_MSG_LEN: usize = 32;
const NUMBER_OF_SENDING_DEVICES: usize = 4;

const RX_BUF_SIZE: usize = 500;
const TX_BUF_SIZE: usize = 30;

static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<TX_BUF_SIZE>> = StaticCell::new();

enum SystemState {
    Standby,
    Online,
}

pub struct ControlLoop<'a, 'd> {
    state: SystemState,
    source_flip_flop: DFlipFlop<'d>,
    sink_ctrl: SinkCtrl<'d>,
    bat_1: Battery<'a, 'd>,
    aux_pwr: AuxPwr<'a>,
    internal_temperature: DynReceiver<'a, i16>,
    can_tranciever: RodosCanInterface<ActivePeriph<'a, NUMBER_OF_SENDING_DEVICES, RODOS_MAX_RAW_MSG_LEN, TX_BUF_SIZE, RX_BUF_SIZE>>
}
impl<'a, 'd> ControlLoop<'a, 'd> {
    pub fn spawn(
        source_flip_flop: DFlipFlop<'d>,
        sink_ctrl: SinkCtrl<'d>,
        bat_1: Battery<'a, 'd>,
        aux_pwr: AuxPwr<'a>,
        internal_temperature: DynReceiver<'a, i16>,
        can_configurator: CanConfigurator<'a>
    ) -> Self {
        // -- CAN configuration
        let mut rodos_can_configurator = RodosCanInterface::new(
            can_configurator,
            RODOS_DEVICE_ID,
        );

        rodos_can_configurator
            .set_bitrate(1_000_000)
            .add_receive_topic(TopicId::Cmd as u16, None).unwrap()
            .add_receive_topic(TopicId::TelemReq as u16, None).unwrap();

        let can_tranciever = rodos_can_configurator.activate(
            TX_BUF.init(TxBuf::<TX_BUF_SIZE>::new()),
            RX_BUF.init(RxBuf::<RX_BUF_SIZE>::new()),
        );

        let state = SystemState::Standby;

        Self { state, source_flip_flop, sink_ctrl, bat_1, aux_pwr, internal_temperature, can_tranciever }
    }

    pub async fn handle_cmd(&mut self, data: &[u8]) {
        match Telecommand::parse(data) {
            Ok(telecommand) => {
                info!("tc: {}", telecommand);
                match telecommand {
                    Telecommand::SetSource(state, time) => {
                        let old_state = self.source_flip_flop.get_state();
                        self.source_flip_flop.set(state).await;
                        if let Some(time) = time {
                            Timer::after_secs(time as u64).await;
                            self.source_flip_flop.set(old_state).await;
                        }
                    },
                    Telecommand::EnableSink(sink, time) => {
                        if self.sink_ctrl.is_enabled(sink) { return; }
                        self.sink_ctrl.enable(sink);
                        if let Some(time) = time {
                            // this is blocking and prevents tm/tc during timeout.
                            // might be good to fix in the future
                            Timer::after_secs(time as u64).await;
                            self.sink_ctrl.disable(sink);
                        }
                    },
                    Telecommand::DisableSink(sink, time) => {
                        if !self.sink_ctrl.is_enabled(sink) { return; }
                        self.sink_ctrl.disable(sink);
                        if let Some(time) = time {
                            // this is blocking and prevents tm/tc during timeout.
                            // might be good to fix in the future
                            Timer::after_secs(time as u64).await;
                            self.sink_ctrl.enable(sink);
                        }
                    },
                }
            },
            Err(e) => error!("{}", e),
        }
    }
    pub async fn send_tm(&mut self) {
        self.can_tranciever.send(TopicId::TelemBat1Tmp as u16,
            &self.bat_1.get_temperature().await.unwrap_or(ERROR_TMP).to_le_bytes()).await
            .unwrap_or_else(|e| error!("could not send bat 1 tmp: {}", e));

        self.can_tranciever.send(TopicId::TelemInternalTmp as u16,
            &self.internal_temperature.get().await.to_le_bytes()).await
            .unwrap_or_else(|e| error!("could not send internal tmp: {}", e));

        self.can_tranciever.send(TopicId::TelemBat1Voltage as u16,
            &self.bat_1.get_voltage().await.to_le_bytes()).await
            .unwrap_or_else(|e| error!("could not send bat 1 voltage: {}", e));

        self.can_tranciever.send(TopicId::TelemAuxPwrVoltage as u16,
            &self.aux_pwr.get_voltage().await.to_le_bytes()).await
            .unwrap_or_else(|e| error!("could not send aux pwr voltage: {}", e));

        let bitmap: u8 = 
            self.source_flip_flop.is_enabled(FlipFlopInput::Bat1) as u8 |
            (self.source_flip_flop.is_enabled(FlipFlopInput::AuxPwr) as u8) << 1 |
            (self.sink_ctrl.is_enabled(Sink::Mainboard) as u8) << 2 |
            (self.sink_ctrl.is_enabled(Sink::RocketLST) as u8) << 3 |
            (self.sink_ctrl.is_enabled(Sink::RocketHD) as u8) << 4;

        self.can_tranciever.send(TopicId::TelemEnableBM as u16,
            &[bitmap]).await
            .unwrap_or_else(|e| error!("could not send enable bm: {}", e));
    }
    
    async fn monitor_ispida_pwr(ispida_pwr: &mut AuxPwr<'_>) {
        loop {
            if ispida_pwr.get_voltage().await >= ACTIVATION_VOLTAGE_THRESHHOLD_10X_MV {
                return;
            }
            Timer::after_millis(100).await;
        }
    }

    async fn run_standby(&mut self) {
        if self.aux_pwr.get_voltage().await < ACTIVATION_VOLTAGE_THRESHHOLD_10X_MV {
            self.state = SystemState::Online;

            self.sink_ctrl.enable(Sink::RocketHD);
            self.sink_ctrl.enable(Sink::RocketLST);
            self.sink_ctrl.enable(Sink::Mainboard);
            return;
        }
        Timer::after_millis(500).await;
    }

    async fn run_online(&mut self) {

        const RODOS_CMD_TOPIC_ID: u16 = TopicId::Cmd as u16;
        const RODOS_TELEM_REQ_TOPIC_ID: u16 = TopicId::TelemReq as u16;

        let event = select(self.can_tranciever.receive(), Self::monitor_ispida_pwr(&mut self.aux_pwr)).await;
        // control over can connection
        match event {
            Either::First(can_result) => match can_result {
                Ok(frame) => {
                    let mut data = [0; RODOS_MAX_RAW_MSG_LEN];
                    data[..frame.data().len()].copy_from_slice(frame.data());
                    match frame.topic() {
                        RODOS_CMD_TOPIC_ID => self.handle_cmd(&data).await,
                        RODOS_TELEM_REQ_TOPIC_ID => self.send_tm().await,
                        _ => error!("impossible topic: {}", frame.topic()),
                    }
                }
                Err(e) => error!("error in frame! {}", e),
            },
            Either::Second(()) => {
                info!("ispida power on, going standby");
                self.state = SystemState::Standby;
                self.sink_ctrl.disable(Sink::RocketHD);
                self.sink_ctrl.disable(Sink::RocketLST);
                self.sink_ctrl.disable(Sink::Mainboard);
            }
        };
    }

    pub async fn run(&mut self) {
        loop {
            match self.state {
                SystemState::Online => self.run_online().await,
                SystemState::Standby => self.run_standby().await,
            };
        }
    }
}
