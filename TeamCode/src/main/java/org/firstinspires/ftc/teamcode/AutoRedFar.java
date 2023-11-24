package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutonomousOp.Alliance.RED;
import static org.firstinspires.ftc.teamcode.AutonomousOp.Zone.FAR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous FAR RED", group="Autonomous")
public class AutoRedFar extends AutonomousOp {
    public Zone getZone() { return FAR; }
    public Alliance getAlliance() { return RED; }
}
