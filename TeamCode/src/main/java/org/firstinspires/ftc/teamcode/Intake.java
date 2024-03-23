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
    public double intake_position = 0.45;
    public double last_intake = 0.0;
    public IntakePlace intake = IntakePlace.Stowed; // we start in stowed
    public boolean override = false;
    public boolean last_right_down = false;
    //public IntakePlace last_intake = IntakePlace.Resting;

    private double timeout = -1.0; // for up/down

    public Intake(HardwareMap hm) {
        suck = new MotorEx(hm, "suck");
        intake_main = new SimpleServo(hm, "intake", 0, 360);
        intake_rev = new SimpleServo(hm, "intake_rev", 0, 360);
        //timer = new Timing.Timer(1706, TimeUnit.MILLISECONDS);
        // 1706 milliseconds is down
    }

    // NOTES:
    // robot-left intake servo (launcher-side): all-left is up, all-right is down (port 3)
    // robot-right intake servo (launcher-side): all-right is up, all-left is down (port 2)
    //

    public void humanInputs(GamepadEx pad, Arm.Position position) {
        // backup (and override!): left joystick puts intake up / down
        if (pad.getLeftY() > 0.5) {
            intake_position += 0.05;
            override = true;
        } else if (pad.getLeftY() < 0.5) {
            intake_position -= 0.05;
            override = true;
        }

        // holding left trigger means: intake in, and suckage
        // (> 0.5 means "is more than half down")
        if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            if (position == Arm.Position.Intake) {
                suck_mode = SuckMode.SUCK;
                intake_position = 1.0;
                override = false;
                last_right_down = false;
            }
        } else {
            if (pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                suck_mode = SuckMode.NOTHING;
                intake_position = 0.45;
                override = false;
                last_right_down = true;
            } else {
                if (!last_right_down) {
                    suck_mode = SuckMode.NOTHING;
                    intake_position = 0.66;
                }
            }
        }

        // debug controls for exact intake position
        if (pad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            intake_position -= 0.05;
            override = true;
        } else if (pad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            intake_position += 0.05;
            override = true;
        }

        // forward/back suckage on intake
        if (pad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            // we do not allow "suck" mode unless the arm is in "intake" position!
            if (position == Arm.Position.Intake) {
                suck_mode = SuckMode.SUCK;
            }
        } else if (pad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            suck_mode = SuckMode.BLOW;
        }
    }

    public void goUp(double time, boolean only_half) {
        intake_position = 1.0;
        if (only_half) {
            intake_position = 0.35;
        }
    }

    public void goDown(double time) {
        goDown(time, 0);
    }
    public void goDown(double time, double delta) {
        intake_position = 0.0;
    }

    void loop(double time) {
        if (intake_position < 0.45) {
            intake_position = 0.45;
        }
        if (intake_position > 1.0) {
            intake_position = 1.0;
        }

/**
        if (intake_place == IntakePlace.Resting) {
            intake_position = 0.5;
        } else if (intake_place == IntakePlace.Intake) {
            intake_position = 1.0;
        } else if (intake == IntakePlace.Stowed) {
            intake_position = 0.0;
        }
**/

        if (intake_position != last_intake) {
            intake_main.setPosition(intake_position);
            intake_rev.setPosition(1.0 - intake_position);
            last_intake = intake_position;
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
