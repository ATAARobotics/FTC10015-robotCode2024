package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class ActionBase {

    abstract boolean update(double time, Drive drive, Telemetry telemetry);
}
