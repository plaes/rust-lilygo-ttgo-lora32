use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayUs, digital::Wait};

use lora_phy::mod_params::{BoardType, RadioError};
use lora_phy::mod_traits::InterfaceVariant;

pub struct LoRaInterfaceVariant<CTRL, WAIT> {
    board_type: BoardType,
    // nss: CTRL,
    reset: CTRL,
    dio0: WAIT,
}

impl<CTRL, WAIT> LoRaInterfaceVariant<CTRL, WAIT>
where
    CTRL: OutputPin,
    WAIT: Wait,
{
    pub fn new(reset: CTRL, dio0: WAIT) -> Result<Self, RadioError> {
        Ok(Self {
            board_type: BoardType::GenericSx1276,
            // nss,
            reset,
            dio0,
        })
    }
}

impl<CTRL, WAIT> InterfaceVariant for LoRaInterfaceVariant<CTRL, WAIT>
where
    CTRL: OutputPin,
    WAIT: Wait,
{
    fn set_board_type(&mut self, board_type: BoardType) {
        self.board_type = board_type
    }

    async fn set_nss_low(&mut self) -> Result<(), RadioError> {
        // self.nss.set_low().map_err(|_| RadioError::NSS)
        Ok(())
    }

    async fn set_nss_high(&mut self) -> Result<(), RadioError> {
        // self.nss.set_high().map_err(|_| RadioError::NSS)
        Ok(())
    }

    async fn reset(&mut self, _delay: &mut impl DelayUs) -> Result<(), RadioError> {
        /*
        TODO
        */
        Ok(())
    }

    async fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        Ok(())
    }

    async fn await_irq(&mut self) -> Result<(), RadioError> {
        self.dio0
            .wait_for_high()
            .await
            .map_err(|_| RadioError::Irq)?;
        Ok(())
    }

    async fn enable_rf_switch_rx(&mut self) -> Result<(), RadioError> {
        Ok(())
    }

    async fn enable_rf_switch_tx(&mut self) -> Result<(), RadioError> {
        Ok(())
    }

    async fn disable_rf_switch(&mut self) -> Result<(), RadioError> {
        Ok(())
    }
}
