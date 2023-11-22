package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionTurn extends ActionBase {

    // use heading-lock to turn to one of the four directions
    double target_heading;
    boolean set_target = false;
    double started = -1;

    ActionTurn(double heading) {
        target_heading = heading;
    }

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        telemetry.addData("target_heaading", target_heading);
        drive.robotInputs(0, 0);
        if (!set_target) {
            drive.odo.pause();
            set_target = true;
            drive.headingControl.setSetPoint(target_heading);
            drive.headingControl.setTolerance(1);
            return false;
        }
        if (drive.headingControl.atSetPoint()) {
            drive.odo.resume(drive.headingControl.getSetPoint());
            return true;
        }
        return false;
    }

    public void draw_field(TelemetryPacket pack){}
}
