package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionInitialIntake extends ActionBase {
    double started = -1;

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0.0) {
            started = time;
            intake.goDown(time, 2.0);
        }
        pack.put("action-intake", started);
        pack.put("action-time", time);
        pack.put("action-mode", intake.rise_mode);
        return intake.rise_mode == Intake.RaisingMode.DO_NOTHING;
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
