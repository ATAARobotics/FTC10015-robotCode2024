package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutonomousOp.Alliance.RED;
import static org.firstinspires.ftc.teamcode.AutonomousOp.Zone.FAR;
import static org.firstinspires.ftc.teamcode.AutonomousOp.Zone.NEAR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Near Blue", group="Autonomous")
public class AutoNearBlue  extends AutonomousOp {
    public Zone getZone() { return NEAR; }
    public Alliance getAlliance() { return RED; }
}
