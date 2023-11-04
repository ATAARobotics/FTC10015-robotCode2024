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

        if (pad.wasJustPressed(GamepadKeys.Button.A)) {
            intake_main.rotateByAngle(1);
        } else if (pad.wasJustPressed(GamepadKeys.Button.B)) {
            intake_main.rotateByAngle(-1);
        }
    }
}
