package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="Test:teleop", group="Opmode")
public class TeleOpTest extends OpMode   {


    @Override
    public void init() {
        // hardwareMap.

    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        telemetry.addData("elapsedTime", "%.3f",time);
        telemetry.update();
    }
}
