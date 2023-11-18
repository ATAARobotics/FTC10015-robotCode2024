package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Scanner;
import java.util.concurrent.TimeUnit;

public class Intake {

    ServoEx intake_main = null;
    ServoEx intake_rev = null;
    MotorEx suck = null;

    public enum RaisingMode {DO_NOTHING, GO_UP, GO_DOWN, GO_TO_TOP, GO_TO_BOTTOM;}
    public enum SuckMode {SUCK, BLOW, NOTHING;}

    public RaisingMode rise_mode = RaisingMode.DO_NOTHING;
    public SuckMode suck_mode = SuckMode.NOTHING;
    public Timing.Timer timer = null;

    public Intake(HardwareMap hm) {
        suck = new MotorEx(hm, "suck");
        intake_main = new SimpleServo(hm, "intake", 0, 360);
        intake_rev = new SimpleServo(hm, "intake_rev", 0, 360);
        timer = new Timing.Timer(1706, TimeUnit.MILLISECONDS);
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

    void loop(double time) {
        switch (rise_mode) {
            case GO_UP:
                intake_main.setPosition(1);
                intake_rev.setPosition(0);
                break;

            case GO_DOWN:
                intake_main.setPosition(0);
                intake_rev.setPosition(1);
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
                suck.set(-1.0);
                break;
            case NOTHING:
                suck.set(0.0);
                break;
        }
    }
}