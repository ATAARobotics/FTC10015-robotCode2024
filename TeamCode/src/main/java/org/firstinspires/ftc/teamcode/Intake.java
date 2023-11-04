package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    MotorEx suck = null;
    public Intake(HardwareMap hm){
        suck = new MotorEx(hm, "suck");
    }

    void update(GamepadEx pad) {
        if (pad.isDown(GamepadKeys.Button.A)) {
            suck.set(0.5);
        } else if (pad.isDown(GamepadKeys.Button.B)) {
            suck.set(-0.5);
        }

    }
}
