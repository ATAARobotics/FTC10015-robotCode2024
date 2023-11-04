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
    public String state;
    public Arm(HardwareMap hm){
        state = "intake";
        arm_main = new MotorEx(hm,"arm_main");
        arm_follower = new MotorEx(hm,"arm_follower");
        arm = new MotorGroup(arm_main,arm_follower);
        arm.resetEncoder();
        arm_control = new PIDController(1,0,0);
        slide = new MotorEx(hm,"slide");
        wrist = new SimpleServo (hm,"wrist");
        claw = new SimpleServo(hm,"claw");
    }
    public void intake(){
        state = "intake";
    }
    public void resting(){
        state = "resting";
    }
    public void scoring(){
        state = "scoring";
    }

    public void update(GamepadEx game){
        if (game.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            arm.set(0.5);
        }
        if (game.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            arm.set(-0.5);
        }

    }
}
