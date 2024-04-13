package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


import java.util.concurrent.TimeUnit;

public class Intake {

    ServoEx intake_main;
    ServoEx intake_rev;
    MotorEx suck;

    ServoController _main_ctl;
    ServoController _rev_ctl;

    public enum SuckMode {SUCK, BLOW, NOTHING}
    public enum IntakePlace {Intake, Resting, Stowed, High}

    public SuckMode suck_mode = SuckMode.NOTHING;
    public double intake_position = 0.45;
    public IntakePlace intake = IntakePlace.Stowed; // we start in stowed
    public boolean full_pizza = false; // when "touch" goes, we're full -- until we "run outwards" once
    public boolean left_was_down = false;

    private double timeout = -1.0; // for up/down

    private double blowing = -1.0; // for 500ms of blow before auto-up
    private double suck_pause = -1.0; // for delay before sucking when coming down

    private double _turning_off = -1.0; // delay before turning servos off

    public Intake(HardwareMap hm) {
        suck = new MotorEx(hm, "suck");
        intake_main = new SimpleServo(hm, "intake", 0, 360);
        intake_rev = new SimpleServo(hm, "intake_rev", 0, 360);
        _main_ctl = hm.get(Servo.class, "intake").getController();
        _rev_ctl = hm.get(Servo.class, "intake_rev").getController();

        //timer = new Timing.Timer(1706, TimeUnit.MILLISECONDS);
        // 1706 milliseconds is down
    }

    public void turn_off_servos() {
        if (_main_ctl.getPwmStatus() != ServoController.PwmStatus.DISABLED) {
            _turning_off = -1.0; // cancel any delayed-off
            _main_ctl.pwmDisable();
            _rev_ctl.pwmDisable();
        }
    }

    public void turn_on_servos() {
        _turning_off = -1.0; // cancel any delayed-off
        if (_main_ctl.getPwmStatus() != ServoController.PwmStatus.ENABLED) {
            _main_ctl.pwmEnable();
            _rev_ctl.pwmEnable();
        }
    }

    // NOTES:
    // robot-left intake servo (launcher-side): all-left is up, all-right is down (port 3)
    // robot-right intake servo (launcher-side): all-right is up, all-left is down (port 2)
    //

    public void humanInputs(GamepadEx pad, double time, Arm.Position position, boolean touch_state) {
        // we saw the touch sensor! our box must be full with two pixels
        boolean just_triggered_full = false;
        boolean wanted_suck_blow = false;

        // XXX FIXME TODO want a boolean to track:
        // - we held left down
        // - touch sensor triggered (so we're full)
        // - don't do "normal left-trigger-down" stuff until we've _released_ it once

        if (left_was_down && pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
            left_was_down = false;
        }

        if (touch_state && !full_pizza) {
            full_pizza = true;
            just_triggered_full = true;
            left_was_down = true;
        }

        if (intake == IntakePlace.Intake){
            //trigger is _not_ down
            if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
                intake = IntakePlace.Resting;
                suck_mode = SuckMode.NOTHING;
            } else {
                if (!full_pizza && suck_pause < 0.0) {
                    suck_mode = SuckMode.SUCK;
                    wanted_suck_blow = true;
                }
            }

            if (just_triggered_full || pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
                blowing = time;
                intake = IntakePlace.Resting;
                suck_mode = SuckMode.BLOW;
                wanted_suck_blow = true;
            }
        } else if (intake == IntakePlace.Stowed || intake == IntakePlace.High) {
            // holding left trigger
            if (!left_was_down && pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                intake = IntakePlace.Intake;
                if (position == Arm.Position.Intake) {
                    if (!full_pizza) {
                        suck_pause = time;
                    } else {
                        suck_mode = SuckMode.NOTHING;
                    }
                }
            }
        } else { // resting
            // holding left trigger
            if (!left_was_down && pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                if (blowing < 0.0) {
                    intake = IntakePlace.Intake;
                }
                if (!full_pizza) {
                    if (position == Arm.Position.Intake) {
                        suck_mode = SuckMode.SUCK;
                        wanted_suck_blow = true;
                    }
                }
            } else if (pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                if (position == Arm.Position.Intake) {
                    intake = IntakePlace.High;
                } else {
                    intake = IntakePlace.Stowed;
                    _turning_off = time + 0.500;
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
        } else if (pad.isDown(GamepadKeys.Button.LEFT_BUMPER )) {
            if (intake == IntakePlace.Resting || intake == IntakePlace.Intake) {
                suck_mode = SuckMode.BLOW;
            }
        } else {
            // want to turn off the suck/blow
            // ... but only if we didn't "want" to do that for other
            // reasons (like left trigger is down)
            if (!wanted_suck_blow) {
                suck_mode = SuckMode.NOTHING;
            }
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

        if (blowing >= 0.0) {
            if (time > blowing + 0.500) {
                intake = IntakePlace.High;
                suck_mode = SuckMode.NOTHING;
                blowing = -1.0;
            } else {
                suck_mode = SuckMode.BLOW;
            }
        }

        if (suck_pause >= 0.0) {
            if (time > suck_pause + .444) {
                suck_mode = SuckMode.SUCK;
                suck_pause = -1.0;
            }
        }

        double old_intake_position = intake_position;
        if (intake == IntakePlace.Resting) {
            intake_position = 0.5;
        } else if (intake == IntakePlace.Intake) {
            intake_position = 1.0;
        } else if (intake == IntakePlace.Stowed) {
            intake_position = 0.30;
        } else if (intake == IntakePlace.High) {
            intake_position = 0.22;
        }

        if (intake_position != old_intake_position) {
            turn_on_servos();
            intake_main.setPosition(intake_position);
            intake_rev.setPosition(1.0 - intake_position);
            if (intake == IntakePlace.Stowed || intake == IntakePlace.High) {
                if (_turning_off < 0.0) {
                    _turning_off = time + 0.500;
                }
            }
        }

        if (_turning_off > 0 && time > _turning_off) {
            turn_off_servos();
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
