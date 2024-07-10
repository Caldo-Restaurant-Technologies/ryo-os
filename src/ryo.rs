use control_components::controllers::clear_core::Controller;

use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::Output;
use crate::config::{
    HATCH_A_CH_A, 
    HATCH_A_CH_B, 
    HATCH_B_CH_A, 
    HATCH_B_CH_B, 
    HATCH_C_CH_A, 
    HATCH_C_CH_B, 
    HATCH_D_CH_A, 
    HATCH_D_CH_B
};
pub fn make_hatches(cc1: Controller, cc2: Controller) -> [Hatch; 4] {

    [
        Hatch::from_io(
            Output::ClearCore(cc1.get_output(HATCH_A_CH_A)), 
            Output::ClearCore(cc1.get_output(HATCH_A_CH_B)),
            cc1.get_analog_input(0)
        ),
        Hatch::from_io(
            Output::ClearCore(cc1.get_output(HATCH_B_CH_A)),
            Output::ClearCore(cc1.get_output(HATCH_B_CH_B)),
            cc1.get_analog_input(0)
        ),
        Hatch::from_io(
            Output::ClearCore(cc2.get_output(HATCH_C_CH_A)),
            Output::ClearCore(cc2.get_output(HATCH_C_CH_B)),
            cc1.get_analog_input(0)
        ),
        Hatch::from_io(
            Output::ClearCore(cc2.get_output(HATCH_D_CH_A)),
            Output::ClearCore(cc2.get_output(HATCH_D_CH_B)),
            cc1.get_analog_input(0)
        ),
    ]
}
