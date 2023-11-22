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

    public enum RaisingMode {DO_NOTHING, GO_UP, GO_DOWN, GO_TO_TOP, GO_TO_BOTTOM}
    public enum SuckMode {SUCK, BLOW, NOTHING}

    public RaisingMode rise_mode = RaisingMode.DO_NOTHING;
    public SuckMode suck_mode = SuckMode.NOTHING;

    private double started_moving = -1.0; // for GO_UP / GO_DOWN

    public Intake(HardwareMap hm) {
        suck = new MotorEx(hm, "suck");
        intake_main = new SimpleServo(hm, "intake", 0, 360);
        intake_rev = new SimpleServo(hm, "intake_rev", 0, 360);
        //timer = new Timing.Timer(1706, TimeUnit.MILLISECONDS);
        // 1706 milliseconds is down
    }

    public void humanInputs(GamepadEx pad) {
        if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            rise_mode = RaisingMode.GO_DOWN;
        } else if (pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            rise_mode = RaisingMode.GO_UP;
        } else {
            rise_mode = RaisingMode.DO_NOTHING;
        }

        // forward/back suckage on intake
        if (pad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            suck_mode = SuckMode.SUCK;
        } else if (pad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            suck_mode = SuckMode.BLOW;
        } else {
            suck_mode = SuckMode.NOTHING;
        }
    }

    public void goUp(double time) {
        rise_mode = RaisingMode.GO_TO_TOP;
        started_moving = time;
    }

    public void goDown(double time) {
        rise_mode = RaisingMode.GO_TO_BOTTOM;
        started_moving = time;
    }

    void loop(double time) {
        if (rise_mode == RaisingMode.GO_TO_TOP || rise_mode == RaisingMode.GO_TO_BOTTOM) {
            double change = time - started_moving;
            //           if (change >= 1.706) {
            if (change >= 1.6) {
                rise_mode = RaisingMode.DO_NOTHING;
            }
        }
        switch (rise_mode) {
            case GO_UP:
            case GO_TO_TOP:
                intake_main.setPosition(0);
                intake_rev.setPosition(1);
                break;

            case GO_DOWN:
            case GO_TO_BOTTOM:
                intake_main.setPosition(1);
                intake_rev.setPosition(0);
                break;

            case DO_NOTHING:
            default:
                intake_main.setPosition(0.5);
                intake_rev.setPosition(0.5);
                break;
        }

        switch (suck_mode) {
            case SUCK:
                suck.set(1.0);
                break;
            case BLOW:
                suck.set(-0.5);
                break;
            case NOTHING:
                suck.set(0.0);
                break;
        }
    }
}