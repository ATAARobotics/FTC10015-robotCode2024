package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public enum Position {Intake, Resting, Scoring, LowScoring};
    MotorEx arm_main;
    MotorEx arm_follower;
    MotorGroup arm;
    PIDController arm_control;
    MotorEx slide;

    ServoEx wrist;
    ServoEx claw;

    Intake intake;
    public Position state;

    double wristp = 0.0;
    double clawp = 0.0; // 0.6 is open, 0.4 (0.37?) is closed

    public Arm(HardwareMap hm){
        state = Position.Intake;
        arm_main = new MotorEx(hm,"arm_main");
        arm_follower = new MotorEx(hm,"arm_follower");
        arm = new MotorGroup(arm_main,arm_follower);
        arm_control = new PIDController(.01,0.01,0);
        arm_control.setTolerance(15);
        // slide = new MotorEx(hm,"slide");
        wrist = new SimpleServo(hm,"wrist", 0, 360);
        claw = new SimpleServo(hm,"claw", 0, 360);
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
        claw.setPosition(1.0);
        arm_control.setSetPoint(0);
        wristp = 0.75;
        clawp = 0.65;
    }
    public void resting(){
        state = Position.Resting;
        arm_control.setSetPoint(-88);
        wristp = 0.4;
        clawp = 0.37;
    }
    public void scoring(){
        state = Position.Scoring;
        arm_control.setSetPoint(-371);
        wristp = 0.6;
    }

    public void low_scoring(){
        state = Position.LowScoring;
        arm_control.setSetPoint(-440);
        wristp = 0.55;
    }
    public void open_claw() {
        clawp = 0.6;
    }

    public void close_claw() {
        clawp = 0.37;
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
/*
         if (game.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            clawp -= 0.1;
            if (clawp < 0.0) {
                clawp = 1.0;
            }
         } else if (game.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
             clawp += 0.1;
            if (clawp > 1.0) {
                clawp = 0.0;
            }
         }
         **/

        // A is close, B is open
        if (game.wasJustPressed(GamepadKeys.Button.A)) {
            if (true || state == Position.Intake) {
                clawp = 0.37; // closed
            }
        } else if (game.wasJustPressed(GamepadKeys.Button.B)){
            if (state == Position.Intake) {
                clawp = 0.6; // open
            } else if (state == Position.Scoring || state == Position.LowScoring) {
                clawp = 0.6; // open
            }
        }
    }

    public void loop(double time){
        double move = arm_control.calculate(arm_main.getCurrentPosition());
        // clamp max speed
        if (move > 0.7) { move = 0.7; }
        if (move < -0.5) { move = -0.5; }
        wrist.setPosition(wristp);
        claw.setPosition(clawp);
        arm.set(move);
        intake.loop(time);
    }
}
