package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="Kiwi: TeleOp", group="Opmode")
public class TeleOpTest extends OpMode   {


    @Override
    public void init() {
        // hardwareMap.

    }

    @Override
    public void start() {
        telemetry.addData("elapsedTime", "%.3f",time);
        telemetry.update();
    }


    @Override
    public void loop() {

    }
}
