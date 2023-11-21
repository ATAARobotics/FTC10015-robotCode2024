package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionIntake extends ActionBase {

    boolean go_up;
    double started = -1.0;

    ActionIntake(boolean up) {
        go_up = up;
    }

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0.0) {
            started = time;
            if (go_up) {
                intake.goUp(time);
            } else {
                intake.goDown(time);
            }
        }
       return intake.rise_mode == Intake.RaisingMode.DO_NOTHING;
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
