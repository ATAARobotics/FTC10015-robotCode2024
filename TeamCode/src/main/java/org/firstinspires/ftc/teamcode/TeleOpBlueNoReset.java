package org.firstinspires.ftc.teamcode;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="no-reset TeleOp Blue", group="Opmode")

public class TeleOpBlueNoReset extends TeleOpBlue {
    @Override
    public void start() {
        plane_launched = false;
        drive.start(180.0);
    }
}
