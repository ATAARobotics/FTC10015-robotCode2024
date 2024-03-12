package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public enum Position {Intake, Resting, HighScoring, MediumScoring, LowScoring, Purple}

    ;

    public enum Roller {Off, In, Out}

    ;

    public enum Climber {Sheathed, Stabby}

    ;
    MotorEx arm_main;
    MotorEx arm_follower;
    MotorGroup arm;
    PIDController arm_control;
    MotorEx slide;

    ServoEx wrist;
    ServoEx roller;
    ServoEx climber;

    RevTouchSensor touch;

    Intake intake;
    public Position state;

    public Climber knives;

    public double climb_countdown_start;

    double wristp = 1.0;
    Roller roller_state = Roller.Off;
    double manual_power = 0.0;

    public Arm(HardwareMap hm) {
        knives = Climber.Sheathed;
        state = Position.Intake;
        arm_main = new MotorEx(hm, "arm_main");
        arm_follower = new MotorEx(hm, "arm_follower");
        arm = new MotorGroup(arm_main, arm_follower);
        arm_control = new PIDController(.02, 0.0, 0.0);
        arm_control.setTolerance(10);
        // slide = new MotorEx(hm,"slide");
        climber = new SimpleServo(hm, "climber", 0, 360);
        wrist = new SimpleServo(hm, "wrist", 0, 360);
        roller = new SimpleServo(hm, "roller", 0, 360);
        touch = hm.get(RevTouchSensor.class, "touch");
        intake = new Intake(hm);
        climb_countdown_start = -1;
        intake();
    }

    public void reset() {
        // reset both motors, for real .. should be done at start of Auto ONLY in normal opreation
        arm_main.stopAndResetEncoder();
        arm_follower.stopAndResetEncoder();
    }

    public void intake() {
        state = Position.Intake;
        //claw.setPosition(1.0);
        arm_control.setSetPoint(0);
        wristp = 1.0;//find out if 0 is left or right
        //roller_state = Roller.In;
    }

    public void resting() {
        state = Position.Resting;
        arm_control.setSetPoint(-88);
        wristp = 0.7;
        roller_state = Roller.Off;
    }

    public void high_scoring() {
        state = Position.HighScoring;
        arm_control.setSetPoint(-330);
        wristp = 0.15;
    }

    public void medium_scoring() {
        state = Position.MediumScoring;
        arm_control.setSetPoint(-360);  // -385 worked better in auto
        wristp = 0.05;
    }

    public void low_scoring() {
        state = Position.LowScoring;
        arm_control.setSetPoint(-385);
        wristp = 0.0;
    }

    public void purple() {
        state = Position.Purple;
        arm_control.setSetPoint(-470);
        wristp = 0.1;
    }

    public void roller_out() {
        roller_state = Roller.Out;
    }

    public void roller_in() {
        roller_state = Roller.In;
    }

    public void roller_off() {
        roller_state = Roller.Off;
    }

    public void humanInputs(GamepadEx game, double time) {
        intake.humanInputs(game, state);
        if (game.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {

            if (state == Position.Purple) {
                wristp += 0.1;
                if (wristp > 1.0) {
                    wristp = 0.0;
                }
            }

            if (state == Position.Intake) {
                resting();
            } else if (state == Position.Resting) {
                high_scoring();
            } else if (state == Position.HighScoring) {
                medium_scoring();
            } else if (state == Position.MediumScoring) {
                low_scoring();
            } // can't go up past "low-scoring"
        } else if (game.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {

            if (state == Position.Purple) {
                wristp -= 0.1;
                if (wristp < 0.0) {
                    wristp = 1.0;
                }
            }

            if (state == Position.Intake) {
                // nothing, intake is lowest
            } else if (state == Position.LowScoring) {
                medium_scoring();
            } else if (state == Position.MediumScoring) {
                high_scoring();
            } else if (state == Position.HighScoring) {
                resting();
            } else if (state == Position.Resting) {
                intake();
            }
        }

        // debug / placement for wrist etc
        if (game.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (true) {
                arm_control.setSetPoint(arm_control.getSetPoint() - 10);
            } else {
                wristp -= 0.1;
                if (wristp < 0.0) {
                    wristp = 1.0;
                }
            }
        } else if (game.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            if (true) {
                arm_control.setSetPoint(arm_control.getSetPoint() + 10);
            } else {
                wristp += 0.1;
                if (wristp > 1.0) {
                    wristp = 0.0;
                }
            }
        }
        /**
         * debugging control
         if (game.getLeftY() < -0.5 ){
         arm_control.setSetPoint(arm_control.getSetPoint() + 1);
         }
         else if (game.getLeftY() > 0.5) {
         arm_control.setSetPoint(arm_control.getSetPoint() - 1);
         }
         **/


        // A is close, B is open
        if (game.isDown(GamepadKeys.Button.A)) {
            roller_state = Roller.In;
        } else if (game.isDown(GamepadKeys.Button.B)) {
            roller_state = Roller.Out;
        } else {
            roller_state = Roller.Off;
        }

        // if we are in knives-out, then mega-power is enabled (so manual_power is only applied then)
        if (game.getRightY() > 0.5 || game.getLeftY() > 0.5) {
            manual_power = 1.0;
        } else {
            manual_power = 0.0;
        }

        /**
         DEBUGGING for purple-arm placer
         if (game.wasJustPressed(GamepadKeys.Button.Y)) {
         if (state == Position.Purple) {
         state = Position.LowScoring;
         } else {
         purple();
         }
         }
         **/

        if (game.isDown(GamepadKeys.Button.LEFT_BUMPER) && state == Position.Intake) {
            roller_state = Roller.In;
        }
        if (game.isDown(GamepadKeys.Button.RIGHT_BUMPER) && state == Position.Intake) {
            roller_state = Roller.In;
        }

        // can "un-climb" with just a quick press
        if (game.wasJustPressed(GamepadKeys.Button.X)) {
            if (knives == Climber.Stabby) {
                knives = Climber.Sheathed;
            }
        }
        // have to hold the button for 0.5 seconds to climb
        if (game.isDown(GamepadKeys.Button.X)) {
            if (climb_countdown_start < 0) {
                climb_countdown_start = time;
            }
            double elapsed = time - climb_countdown_start;
            if (knives == Climber.Sheathed && elapsed > 0.5) {
                knives = Climber.Stabby;
                arm_control.setSetPoint(-250);
                wristp = 1.0;
                climb_countdown_start = -1;
            }
        }
    }

    public void loop(double time){
        double move = arm_control.calculate(arm_main.getCurrentPosition());
         //clamp max speed
        if (move > 0.7) { move = 0.7; }
        if (move < -0.8) { move = -0.8; }
        wrist.setPosition(wristp);

        // roller logic
        double roller_pos = 0.5;
        if (roller_state == Roller.In && !touch.isPressed() ) {
            roller_pos = 1.0;
        } else if (roller_state == Roller.Out) {
            roller_pos = 0.0;
        } else {
            roller_pos = 0.5;
        }
        // override: if touch sensor is on, no roller movement
        roller.setPosition(roller_pos);

        // climber thing
        double final_move = move;
        if (knives == Climber.Stabby) {
            // XXX TODO don't deploy if arm is "too low"
            climber.setPosition(1.0);
            // in this mode, we have "full-power mega-climb" enabled
            if (manual_power != 0.0) {
                final_move = manual_power;
            }
        } else {
            climber.setPosition(0.0);
        }
        arm.set(final_move);
        intake.loop(time);
    }
}
