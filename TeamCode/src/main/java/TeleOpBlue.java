import static org.firstinspires.ftc.teamcode.TeleOp.Alliance.BLUE;

import org.firstinspires.ftc.teamcode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp Blue", group="Opmode")
public class TeleOpBlue extends TeleOp {

    public Alliance getAlliance() {return BLUE;}


    // no reset code-path
    /*
    @Override
    public void start() {
        plane_launched = false;
        drive.start();
    }

     */
}
