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

    public enum Position {Intake, Resting, Scoring, LowScoring};
    public enum Roller {Off, In, Out};
    public enum Climber {Sheathed, Stabby};
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

    double wristp = 1.0;
    Roller roller_state = Roller.Off;
    double manual_power = 0.0;

    public Arm(HardwareMap hm){
        knives = Climber.Sheathed;
        state = Position.Intake;
        arm_main = new MotorEx(hm,"arm_main");
        arm_follower = new MotorEx(hm,"arm_follower");
        arm = new MotorGroup(arm_main,arm_follower);
        arm_control = new PIDController(.01,0.01,0);
        arm_control.setTolerance(15);
        // slide = new MotorEx(hm,"slide");
        climber = new SimpleServo(hm,"climber", 0, 360);
        wrist = new SimpleServo(hm,"wrist", 0, 360);
        roller = new SimpleServo(hm,"roller", 0, 360);
        touch = hm.get(RevTouchSensor.class , "touch");
        intake = new Intake(hm);
        intake();
    }

    public void reset() {
        // reset both motors, for real .. should be done at start of Auto ONLY in normal opreation
        arm_main.stopAndResetEncoder();
        arm_follower.stopAndResetEncoder();
    }
    public void intake(){
        state = Position.Intake;
        //claw.setPosition(1.0);
        arm_control.setSetPoint(0);
        wristp = 1.0;//find out if 0 is left or right
        roller_state = Roller.In;
    }
    public void resting(){
        state = Position.Resting;
        arm_control.setSetPoint(-88);
        wristp = 0.7;
        roller_state = Roller.Off;
    }
    public void scoring(){
        state = Position.Scoring;
        arm_control.setSetPoint(-300);
        wristp = 0.0;
    }

    public void low_scoring(){
        state = Position.LowScoring;
        arm_control.setSetPoint(-370);
        wristp = 0.0;
    }
    public void roller_out() {
        roller_state = Roller.Out;
    }

    public void roller_in() {
        roller_state = Roller.In;
    }

    public void humanInputs(GamepadEx game){
        intake.humanInputs(game, state);
        if (game.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if (state == Position.Intake) {
                resting();
            } else if (state == Position.Scoring) {
                low_scoring();
            } else if (state == Position.Resting) {
                scoring();
            } // can't go up past "low-scoring"
        } else if (game.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            if (state == Position.Intake) {
                // nothing, intake is lowest
            } else if (state == Position.LowScoring) {
                scoring();
            } else if (state == Position.Scoring) {
                resting();
            } else if (state == Position.Resting){
                intake();
            }
        }

        // debug / placement for wrist etc
        if (game.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            wristp -= 0.1;
            if (wristp < 0.0) {
                wristp = 1.0;
            }
        } else if (game.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            wristp += 0.1;
            if (wristp > 1.0) {
                wristp = 0.0;
            }
        }
        if (game.getLeftY() < -0.5 ){
            arm_control.setSetPoint(arm_control.getSetPoint() + 1);
        }
        else if (game.getLeftY() > 0.5) {
            arm_control.setSetPoint(arm_control.getSetPoint() - 1);
        }


        // A is close, B is open
        if (game.isDown(GamepadKeys.Button.A)) {
            roller_state = Roller.In;
        } else if (game.isDown(GamepadKeys.Button.B)){
            roller_state = Roller.Out;
        }
        else {
            roller_state = Roller.Off;
        }

        // if we are in knives-out, then

        if (game.isDown(GamepadKeys.Button.LEFT_BUMPER) && state == Position.Intake){
            roller_state = Roller.In;
        }
        if (game.wasJustPressed(GamepadKeys.Button.X)){
            if (knives == Climber.Sheathed){
                knives = Climber.Stabby;
                arm_control.setSetPoint(-237);
                wristp = 1.0;
            } else {
                knives = Climber.Sheathed;
            }
        }
    }

    public void loop(double time){
        double move = arm_control.calculate(arm_main.getCurrentPosition());
         //clamp max speed
        if (move > 0.7) { move = 0.7; }
        if (move < -0.5) { move = -0.5; }
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

        } else {
            climber.setPosition(0.0);
        }
        arm.set(move);
        intake.loop(time);
    }
}
