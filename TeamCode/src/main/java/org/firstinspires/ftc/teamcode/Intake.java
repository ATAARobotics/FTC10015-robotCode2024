package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    ServoEx intake_main = null;
    MotorEx suck = null;
    public Intake(HardwareMap hm){
        suck = new MotorEx(hm, "suck");
        intake_main = new SimpleServo(hm, "intake", 0, 360);
    }

    void update(GamepadEx pad) {
        if (pad.isDown(GamepadKeys.Button.A)) {
            suck.set(0.5);
        } else if (pad.isDown(GamepadKeys.Button.B)) {
            suck.set(-0.5);
        } else {
            suck.set(0.0);
        }

        // left/right triggers put intake up or down
        if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            intake_main.setPosition(0.5 - pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        } else if (pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            intake_main.setPosition(pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else {
            intake_main.setPosition(0.5);
        }

        // X + Y for forward/back suckage
        if (pad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            suck.set(1.0);
        } else if (pad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            suck.set(-1.0);
        } else {
            suck.set(0.0);
        }
    }
}
