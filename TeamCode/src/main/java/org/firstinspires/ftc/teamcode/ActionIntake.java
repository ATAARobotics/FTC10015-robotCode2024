package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionIntake extends ActionBase {

    boolean go_up;
    double started = -1;

    ActionIntake(boolean up) {
        go_up = up;
    }

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0.0) {
            started = time;
            if (go_up) {
                intake.intake_main.setPosition(0.0);
                intake.intake_rev.setPosition(1.0);
            } else {
                intake.intake_main.setPosition(1.0);
                intake.intake_rev.setPosition(0.0);
            }
        }
        double elapsed = time - started;
        if (elapsed > 1.0) {
            intake.intake_main.setPosition(0.5);
            intake.intake_rev.setPosition(0.5);
            // if we're "going down" then we spit out at the end too .. probably separate action?
            if (!go_up) {
                intake.suck.set(0.3);
            } else {
                return true;
            }
        }
        if (elapsed > 1.5) {
            intake.suck.set(0.0);
            return true;
        }
        return false;
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
