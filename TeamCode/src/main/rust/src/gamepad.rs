use javawithrust::prelude::*;


#[jclass(com.qualcomm.robotcore.hardware.Gamepad)]
pub struct Gamepad {
    a                  : bool,
    b                  : bool,
    back               : bool,
    dpad_down          : bool,
    dpad_left          : bool,
    dpad_right         : bool,
    dpad_up            : bool,
    dpadThreshold      : f32,
    guide              : bool,
    id                 : i32,
    joystickDeadzone   : f32,
    left_bumper        : bool,
    left_stick_button  : bool,
    left_stick_x       : f32,
    left_stick_y       : f32,
    left_trigger       : f32,
    right_bumper       : bool,
    right_stick_button : bool,
    right_stick_x      : f32,
    right_stick_y      : f32,
    right_trigger      : f32,
    start              : bool,
    timestamp          : f64,
    x                  : bool,
    y                  : bool
}
