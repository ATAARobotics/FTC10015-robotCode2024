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
    public enum IntakePlace {Intake, Resting, Stowed, High}

    public SuckMode suck_mode = SuckMode.NOTHING;
    public double intake_position = 0.45;
    public IntakePlace intake = IntakePlace.Stowed; // we start in stowed
    public boolean full_pizza = false; // when "touch" goes, we're full -- until we "run outwards" once

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

    public void humanInputs(GamepadEx pad, Arm.Position position, boolean touch_state) {
        // we saw the touch sensor! our box must be full with two pixels
        boolean just_triggered_full = false;
        if (touch_state && !full_pizza) {
            full_pizza = true;
            just_triggered_full = true;
        }

        if (intake == IntakePlace.Intake){
            //trigger is _not_ down
            if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
                intake = IntakePlace.Resting;
                suck_mode = SuckMode.NOTHING;
            }
            if (just_triggered_full || pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
                intake = IntakePlace.Stowed;
                suck_mode = SuckMode.NOTHING;
            }
        } else if (intake == IntakePlace.Stowed) {
            // holding left trigger

            if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                intake = IntakePlace.Intake;
                if (position == Arm.Position.Intake) {
                    if (!full_pizza) {
                        suck_mode = SuckMode.SUCK;
                    } else {
                        suck_mode = SuckMode.NOTHING;
                    }
                }
            }
        } else { // resting
            // holding left trigger
            if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                intake = IntakePlace.Intake;
                if (position == Arm.Position.Intake) {
                    if (!full_pizza) {
                        suck_mode = SuckMode.SUCK;
                    } else {
                        suck_mode = SuckMode.NOTHING;
                    }
                } else {
                    suck_mode = SuckMode.NOTHING;
                }
            } else if (full_pizza || pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                if (position == Arm.Position.Intake) {
                    intake = IntakePlace.High;
                } else {
                    intake = IntakePlace.Stowed;
                }
                suck_mode = SuckMode.NOTHING;
            }
        }

        // kind of janky -- we're MIRRORing the control that Arm uses
        // here; would be better if it was done there?
        if (pad.isDown(GamepadKeys.Button.B)) {
            // we're running the out-take at least once, so we don't
            // _KNOW_ we're full anymore
            full_pizza = false;
        }

        // forward/back suckage on intake
        if (pad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            // we do not allow "suck" mode unless the arm is in "intake" position!
            if (position == Arm.Position.Intake) {
                suck_mode = SuckMode.SUCK;
            }
        } else if (pad.isDown(GamepadKeys.Button.LEFT_BUMPER )&& (intake == IntakePlace.Resting || intake == IntakePlace.Intake)) {
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
//        intake_position = 0.0;
    }

    public void arm_moving() {
        if (intake == IntakePlace.High) {
            intake = IntakePlace.Stowed;
        }
    }

    void loop(double time) {
/*
        if (intake_position < 0.0) {
            intake_position = 0.0;
        }
        if (intake_position > 1.0) {
            intake_position = 1.0;
        }
*/


        if (intake == IntakePlace.Resting) {
            intake_position = 0.5;
        } else if (intake == IntakePlace.Intake) {
            intake_position = 1.0;
        } else if (intake == IntakePlace.Stowed) {
            intake_position = 0.30;
        } else if (intake == IntakePlace.High) {
            intake_position = 0.22;
        }

        intake_main.setPosition(intake_position);
        intake_rev.setPosition(1.0 - intake_position);

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
