package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionPause extends ActionBase {

    double started = -1;
    double length;
    ActionPause(double how_long) {
        length = how_long;
    }
    boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0) {
            started = time;
            return false;
        }
        return (started + length) < time;
    }

    void draw_field(TelemetryPacket pack) {}
}
