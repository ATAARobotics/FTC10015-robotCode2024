package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutonomousOp.Alliance.RED;
import static org.firstinspires.ftc.teamcode.AutonomousOp.Zone.FAR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Near RED", group="Autonomous")
public class AutoRedNear extends AutonomousOp {
    public Zone getZone() { return Zone.NEAR; }
    public Alliance getAlliance() { return RED; }
}

