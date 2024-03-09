@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="no-reset TeleOp RED", group="Opmode")

public class TeleOpRedNoReset extends TeleOpRed {
    @Override
    public void start() {
        plane_launched = false;
        drive.start();
    }
}
