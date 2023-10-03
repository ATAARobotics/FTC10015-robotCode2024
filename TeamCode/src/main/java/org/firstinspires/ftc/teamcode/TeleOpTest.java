package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="Test:teleop", group="Opmode")
public class TeleOpTest extends OpMode   {

    public DcMotor motor_fl = null;
    public DcMotor motor_fr = null;
    public DcMotor motor_bl = null;
    public DcMotor motor_br = null;



    @Override
    public void init() {
        //hardwareMap.Set
        motor_fl = (DcMotor) hardwareMap.get("FL_Drive");
        motor_fr = (DcMotor) hardwareMap.get("FR_Drive");
        motor_bl = (DcMotor) hardwareMap.get("BL_Drive");
        motor_br = (DcMotor) hardwareMap.get("BR_Drive");
    }

    @Override
    public void start() {


    }


    @Override
    public void loop() {
        telemetry.addData("elapsedTime", "%.3f",time);
        if (gamepad1.a) {
            motor_fl.setPower(0.2);
            //positive power is "backwards" relative to the robot
        } else {
            motor_fl.setPower(0);
        }
        telemetry.update();
    }

}
