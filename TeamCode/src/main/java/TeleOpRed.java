import static org.firstinspires.ftc.teamcode.TeleOp.Alliance.RED;

import org.firstinspires.ftc.teamcode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp RED", group="Opmode")
public class TeleOpRed extends TeleOp
{
    public Alliance getAlliance() { return RED; }


    // no reset code-path
    /*
    @Override
    public void start() {
        plane_launched = false;
        drive.start();
    }
    */
}
