package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionSuck extends ActionBase {

    boolean suck = true;
    double target_time = 1.5;
    double started = -1;

    ActionSuck(boolean suck) {
        this.suck = suck;
        this.target_time = 1.5;
    }
    ActionSuck(boolean suck, double time)
    {
        this.suck = suck;
        this.target_time = time;
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
        pack.put("suck-elapsed", (time - started));
        if ((time - started) > target_time) {
            intake.suck_mode = Intake.SuckMode.NOTHING;
            return true;
        }
        return false;
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
