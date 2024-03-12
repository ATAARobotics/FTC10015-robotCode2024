package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

public class Intake {

    ServoEx intake_main;
    ServoEx intake_rev;
    MotorEx suck;

    public enum SuckMode {SUCK, BLOW, NOTHING}
    public enum IntakePlace {Intake, Resting, Stowed}

    public SuckMode suck_mode = SuckMode.NOTHING;
    public double intake_position = 0.5; // XXX what are min / max positions?
    public IntakePlace intake = Stowed; // we start in stowed

    private double timeout = -1.0; // for up/down

    public Intake(HardwareMap hm) {
        suck = new MotorEx(hm, "suck");
        intake_main = new SimpleServo(hm, "intake", 0, 360);
        intake_rev = new SimpleServo(hm, "intake_rev", 0, 360);
        //timer = new Timing.Timer(1706, TimeUnit.MILLISECONDS);
        // 1706 milliseconds is down
    }

    public void humanInputs(GamepadEx pad, Arm.Position position) {
        if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            intake_position -= 0.05;
        } else if (pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            intake_position += 0.05;
        }

        // forward/back suckage on intake
        if (pad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            // we do not allow "suck" mode unless the arm is in "intake" position!
            if (position == Arm.Position.Intake) {
                suck_mode = SuckMode.SUCK;
            }
        } else if (pad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            suck_mode = SuckMode.BLOW;
        } else {
            suck_mode = SuckMode.NOTHING;
        }
    }

    public void goUp(double time, boolean only_half) {
        intake_position = 1.0;
    }

    public void goDown(double time) {
        goDown(time, 0);
    }
    public void goDown(double time, double delta) {
        intake_position = 0.0;
    }

    void loop(double time) {
        if (intake_position < 0.3) {
            intake_position = 0.3;
        }
        if (intake_position > 0.8) {
            intake_position = 0.8;
        }

        intake_main.setPosition(intake_position);
        intake_rev.setPosition(1.0 - intake_position);

        switch (suck_mode) {
            case SUCK:
                suck.set(1.0);
                break;
            case BLOW:
                suck.set(-0.42);
                break;
            case NOTHING:
                suck.set(0.0);
                break;
        }
    }
}
