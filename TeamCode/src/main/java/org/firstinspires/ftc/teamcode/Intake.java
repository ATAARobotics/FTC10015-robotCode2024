package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

public class Intake {

    ServoEx intake_main = null;
    ServoEx intake_rev = null;
    MotorEx suck = null;

    boolean position_up = true;
    boolean desired_position = true;
    double started_moving = -1;

    public Timing.Timer timer = null;
    public Intake(HardwareMap hm){
        suck = new MotorEx(hm, "suck");
        intake_main = new SimpleServo(hm, "intake", 0, 360);
        intake_rev = new SimpleServo(hm, "intake_rev", 0, 360);
        timer = new Timing.Timer(0, TimeUnit.MILLISECONDS);
        // 1706 microseconds is down
    }

    public void go_down(double time) {
        if (started_moving < 0.0) {
            desired_position = false;
            if (desired_position != position_up) {
                started_moving = time;
                intake_main.setPosition(0);
                intake_rev.setPosition(1.0);
            }
        }
    }

    public void go_up(double time) {
        if (started_moving < 0.0) {
            desired_position = true;
            if (desired_position != position_up) {
                started_moving = time;
                intake_main.setPosition(1.0);
                intake_rev.setPosition(0.0);
            }
        }
    }

    void update(double time, GamepadEx pad) {
        if (pad.isDown(GamepadKeys.Button.A)) {
            suck.set(0.5);
        } else if (pad.isDown(GamepadKeys.Button.B)) {
            suck.set(-0.5);
        } else {
            suck.set(0.0);
        }

        // left/right triggers put intake up or down
        if (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            go_up(time);
            //intake_main.setPosition(0);
            //intake_rev.setPosition(1);
        } else if (pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            go_down(time);
            //intake_main.setPosition(1);
            //intake_rev.setPosition(0);
        }

        // forward/back suckage on intake
        if (pad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            suck.set(1.0);
        } else if (pad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            suck.set(-1.0);
        } else {
            suck.set(0.0);
        }

        if (started_moving >= 0.0) {
            if (time - started_moving >= 1.706) {
                intake_main.setPosition(0.5);
                intake_rev.setPosition(0.5);
                started_moving = -1;
            }
        }
    }
}
