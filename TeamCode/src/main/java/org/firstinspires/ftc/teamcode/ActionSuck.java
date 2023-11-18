package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionSuck extends ActionBase {

    boolean suck = true;
    double started = -1;

    ActionSuck(boolean suck) {
        this.suck = suck;
    }

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0.0) {
            started = time;
            if (suck) {
                intake.suck_mode = Intake.SuckMode.SUCK;
            } else {
                intake.suck_mode = Intake.SuckMode.BLOW;
            }
            return false;
        }
        return (time - started) > 0.3;
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
