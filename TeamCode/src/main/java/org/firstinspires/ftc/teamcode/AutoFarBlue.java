package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutonomousOp.Alliance.RED;
import static org.firstinspires.ftc.teamcode.AutonomousOp.Zone.FAR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous FAR Blue", group="Autonomous")
public class AutoFarBlue  extends AutonomousOp {
    public Zone getZone() { return Zone.FAR; }
    public Alliance getAlliance() { return Alliance.BLUE; }
}
