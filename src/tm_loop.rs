

    pub async fn send_tm(&mut self) {
        self.can_tranciever
            .send(
                tm::eps::Bat1Temperature::ID as u16,
                &self
                    .bat_1
                    .get_temperature()
                    .await
                    .unwrap_or(ERROR_TMP)
                    .to_le_bytes(),
            )
            .await
            .unwrap_or_else(|e| error!("could not send bat 1 tmp: {}", e));

        self.can_tranciever
            .send(
                tm::eps::InternalTemperature::ID as u16,
                &self.internal_temperature.get().await.to_le_bytes(),
            )
            .await
            .unwrap_or_else(|e| error!("could not send internal tmp: {}", e));

        self.can_tranciever
            .send(
                tm::eps::Bat1Voltage::ID as u16,
                &self.bat_1.get_voltage().await.to_le_bytes(),
            )
            .await
            .unwrap_or_else(|e| error!("could not send bat 1 voltage: {}", e));

        self.can_tranciever
            .send(
                tm::eps::AuxPowerVoltage::ID as u16,
                &self.aux_pwr.get_voltage().await.to_le_bytes(),
            )
            .await
            .unwrap_or_else(|e| error!("could not send aux pwr voltage: {}", e));

        let bitmap: u8 = self.source_flip_flop.is_enabled(FlipFlopInput::Bat1) as u8
            | (self.source_flip_flop.is_enabled(FlipFlopInput::AuxPwr) as u8) << 1
            | (self.sink_ctrl.is_enabled(Sink::Mainboard) as u8) << 2
            | (self.sink_ctrl.is_enabled(Sink::RocketLST) as u8) << 3
            | (self.sink_ctrl.is_enabled(Sink::RocketHD) as u8) << 4;

        self.can_tranciever
            .send(tm::eps::EnableBitmap::ID as u16, &[bitmap])
            .await
            .unwrap_or_else(|e| error!("could not send enable bm: {}", e));
    }
