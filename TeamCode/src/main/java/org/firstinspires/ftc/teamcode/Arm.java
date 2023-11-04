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
    MotorEx arm_main = null;
    MotorEx arm_follower = null;
    MotorGroup arm = null;
    PIDController arm_control = null;
    MotorEx slide = null;

    ServoEx wrist = null;
    ServoEx claw = null;

    Intake intake = null;
    public String state;
    public Arm(HardwareMap hm){
        state = "intake";
        arm_main = new MotorEx(hm,"arm_main");
        arm_follower = new MotorEx(hm,"arm_follower");
        arm = new MotorGroup(arm_main,arm_follower);
        arm.resetEncoder();
        arm_control = new PIDController(.01,0,0);
        // slide = new MotorEx(hm,"slide");
        wrist = new SimpleServo(hm,"wrist", 0, 360);
        claw = new SimpleServo(hm,"claw", 0, 360);
        intake = new Intake(hm);
    }
    public void intake(){
        state = "intake";
        arm_control.setSetPoint(0);
    }
    public void resting(){
        state = "resting";
        arm_control.setSetPoint(-65);
    }
    public void scoring(){
        state = "scoring";
        arm_control.setSetPoint(-360);
    }

    public void update(GamepadEx game){
        if (game.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            if (state == "intake") {
                resting();
            } else if (state == "scoring") {
                // nothing, can't go past scoring
            } else if (state == "resting") {
                scoring();
            }
        } else if (game.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            if (state == "intake") {
                // nothing, intake is lowest
            } else if (state == "scoring") {
                resting();
            } else if (state == "resting"){
                intake();
            }
        }

        double move = arm_control.calculate(arm_main.getCurrentPosition());
        arm.set(move);
        intake.update(game);
    }
}
