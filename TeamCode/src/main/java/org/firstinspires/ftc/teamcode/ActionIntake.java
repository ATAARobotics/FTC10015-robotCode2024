package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionIntake extends ActionBase {

    boolean go_up;
    boolean half;
    double started = -1.0;

    ActionIntake(boolean up) {
        go_up = up;
        half = false;
    }
    ActionIntake(boolean up, boolean up_half) {
        go_up = up;
        half = up_half;
    }

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0.0) {
            started = time;
            if (go_up) {
                intake.goUp(time, half);
            } else {
                intake.goDown(time);
            }
        }
        pack.put("action-intake", started);
        pack.put("action-time", time);
        pack.put("action-mode", intake.rise_mode);
       return intake.rise_mode == Intake.RaisingMode.DO_NOTHING;
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
