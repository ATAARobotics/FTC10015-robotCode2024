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
    MotorEx arm_main;
    MotorEx arm_follower;
    MotorGroup arm;
    PIDController arm_control;
    MotorEx slide;

    ServoEx wrist;
    ServoEx claw;

    Intake intake;
    public String state;

    double wristp = 0.0;
    double clawp = 0.0; // 0.6 is open, 0.4 (0.37?) is closed

    public Arm(HardwareMap hm){
        state = "intake";
        arm_main = new MotorEx(hm,"arm_main");
        arm_follower = new MotorEx(hm,"arm_follower");
        arm = new MotorGroup(arm_main,arm_follower);
        arm.resetEncoder();
        arm_control = new PIDController(.01,0.01,0);
        arm_control.setTolerance(15);
        // slide = new MotorEx(hm,"slide");
        wrist = new SimpleServo(hm,"wrist", 0, 360);
        claw = new SimpleServo(hm,"claw", 0, 360);
        intake = new Intake(hm);
        state = "resting";
        intake();
    }
    public void intake(){
        state = "intake";
        claw.setPosition(1.0);
        arm_control.setSetPoint(0);
        wristp = 0.75;
        clawp = 0.65;
    }
    public void resting(){
        state = "resting";
        arm_control.setSetPoint(-88);
        wristp = 0.4;
        clawp = 0.37;
    }
    public void scoring(){
        state = "scoring";
        arm_control.setSetPoint(-360);
        wristp = 0.6;
    }

    public void low_scoring(){
        state = "low-scoring";
        arm_control.setSetPoint(-440);
        wristp = 0.5;
    }
    public void open_claw() {
        clawp = 0.6;
    }

    public void close_claw() {
        clawp = 0.37;
    }

    public void humanInputs(GamepadEx game){
        intake.humanInputs(game);
        if (game.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if (state.equals("intake")) {
                resting();
            } else if (state.equals("scoring")) {
                low_scoring();
            } else if (state.equals("resting")) {
                scoring();
            } // can't go up past "low-scoring"
        } else if (game.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            if (state == "intake") {
                // nothing, intake is lowest
            } else if (state == "low-scoring"){
                scoring();
            } else if (state == "scoring") {
                resting();
            } else if (state == "resting"){
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
            if (true || state == "intake") {
                clawp = 0.37; // closed
            }
        } else if (game.wasJustPressed(GamepadKeys.Button.B)){
            if (state == "intake") {
                clawp = 0.6; // open
            } else if (state == "scoring" || state == "low-scoring") {
                clawp = 0.6; // open
            }
        }

        wrist.setPosition(wristp);
        claw.setPosition(clawp);
    }

    public void loop(double time){
        double move = arm_control.calculate(arm_main.getCurrentPosition());
        // clamp max speed
        if (move > 0.7) { move = 0.7; }
        if (move < -0.5) { move = -0.5; }
        arm.set(move);
        intake.loop(time);
    }
}
