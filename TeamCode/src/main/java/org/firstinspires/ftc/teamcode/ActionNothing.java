package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionNothing extends ActionBase {

    boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        drive.robotInputs(0, 0);
        return false;
    }

    void draw_field(TelemetryPacket pack) {
    }
}