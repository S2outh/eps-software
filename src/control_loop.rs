mod telecommands;

use defmt::{error, info};
use embassy_stm32::can::{CanConfigurator, RxBuf, TxBuf};
use embassy_sync::watch::DynReceiver;
use rodos_can_interface::{ActivePeriph, RodosCanInterface};
use static_cell::StaticCell;

use crate::control_loop::telecommands::Telecommand;
use crate::pwr_src::d_flip_flop::DFlipFlop;
use crate::pwr_src::sink_ctrl::SinkCtrl;

use super::pwr_src::battery::Battery;
use super::pwr_src::aux_pwr::AuxPwr;

const RODOS_DEVICE_ID: u8 = 0x02;

const RODOS_CMD_TOPIC_ID: u16 = 8000;

const RODOS_TELEM_REQ_TOPIC_ID: u16 = 1000;

const RODOS_TELEM_ENABLE_BM_TOPIC_ID: u16 = 1410;
const RODOS_TELEM_BAT_1_TMP_TOPIC_ID: u16 = 1411;
const RODOS_TELEM_INERNAL_TMP_TOPIC_ID: u16 = 1412;
const RODOS_TELEM_BAT_1_VOLTAGE_TOPIC_ID: u16 = 1413;
const RODOS_TELEM_AUX_PWR_VOLTAGE_TOPIC_ID: u16 = 1414;

const RODOS_MAX_RAW_MSG_LEN: usize = 32;
const NUMBER_OF_SENDING_DEVICES: usize = 4;

const RX_BUF_SIZE: usize = 500;
const TX_BUF_SIZE: usize = 30;

static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<TX_BUF_SIZE>> = StaticCell::new();

pub struct ControlLoop<'a, 'd> {
    source_flip_flop: DFlipFlop<'d>,
    sink_ctrl: SinkCtrl<'d>,
    bat_1: Battery<'a, 'd>,
    aux_pwr: AuxPwr<'a, 'd>,
    internal_temperature: DynReceiver<'a, i16>,
    can_tranciever: RodosCanInterface<ActivePeriph<'a, NUMBER_OF_SENDING_DEVICES, RODOS_MAX_RAW_MSG_LEN, TX_BUF_SIZE, RX_BUF_SIZE>>
}
impl<'a, 'd> ControlLoop<'a, 'd> {
    pub fn spawn(
        source_flip_flop: DFlipFlop<'d>,
        sink_ctrl: SinkCtrl<'d>,
        bat_1: Battery<'a, 'd>,
        aux_pwr: AuxPwr<'a, 'd>,
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
            .add_receive_topic(RODOS_CMD_TOPIC_ID, None).unwrap()
            .add_receive_topic(RODOS_TELEM_REQ_TOPIC_ID, None).unwrap();

        let can_tranciever = rodos_can_configurator.activate(
            TX_BUF.init(TxBuf::<TX_BUF_SIZE>::new()),
            RX_BUF.init(RxBuf::<RX_BUF_SIZE>::new()),
        );

        Self { source_flip_flop, sink_ctrl, bat_1, aux_pwr, internal_temperature, can_tranciever }
    }

    pub async fn handle_cmd(&mut self, data: &[u8]) {
        info!("received tc");
        match Telecommand::parse(data) {
            Ok(telecommand) => match telecommand {
                Telecommand::SetSource(state) => self.source_flip_flop.set(state).await,
                Telecommand::EnableSink(sink) => self.sink_ctrl.enable(sink),
                Telecommand::DisableSink(sink) => self.sink_ctrl.disable(sink),
            },
            Err(e) => error!("{}", e),
        }
    }

    pub async fn run(&mut self) {
        loop {
            match self.can_tranciever.receive().await {
                Ok(frame) => {
                    let mut data = [0; RODOS_MAX_RAW_MSG_LEN];
                    data[..frame.data().len()].copy_from_slice(frame.data());
                    match frame.topic() {
                        RODOS_CMD_TOPIC_ID => self.handle_cmd(&data).await,
                        RODOS_TELEM_REQ_TOPIC_ID => {},
                        _ => error!("impossible topic: {}", frame.topic()),
                    }
                                    }
                Err(e) => error!("error in frame! {}", e),
            };
        }
    }
}
