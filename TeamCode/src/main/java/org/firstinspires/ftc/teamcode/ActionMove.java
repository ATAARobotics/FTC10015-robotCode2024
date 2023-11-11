package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionMove extends ActionBase {

    double target_x;
    double target_y;
    PIDController control_x = null;

    PIDController control_y = null;

    ActionMove(double x, double y) {
        control_x = new PIDController(0.015, 0.0002,0);
        control_y = new PIDController(0.015, 0.0002, 0);
        control_x.setTolerance(4);
        control_y.setTolerance(4);
        set_target(x, y);
    }

    public void set_target(double x, double y) {
        target_x = x;
        target_y = y;
        control_x.setSetPoint(target_x);
        control_y.setSetPoint(target_y);
    }
    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        double strafe = control_x.calculate(drive.odo.position_x());
        double forward = control_y.calculate(drive.odo.position_y());
        telemetry.addData("strafe", strafe);
        telemetry.addData("forward", forward);
        telemetry.addData("x", drive.odo.position_x());
        telemetry.addData("y", drive.odo.position_y());
        telemetry.addData("at_target", at_target(drive));

        pack.put("strafe", strafe);
        pack.put("forward", forward);
        pack.put("x", drive.odo.position_x());
        pack.put("y", drive.odo.position_y());
        pack.put("at_target", at_target(drive));

        if (at_target(drive)) {
            drive.robotInputs(0, 0);
            return true;
        }
        drive.robotInputs(strafe, forward);
        return false;
    }

    public void draw_field(TelemetryPacket pack) {
        pack.fieldOverlay()
                .setFill("blue")
                .fillCircle(target_x, target_y, 2.0);
    }

    public boolean at_target(Drive drive) {
        return (control_y.atSetPoint() && control_x.atSetPoint());/*
        double x_err = Math.abs(drive.odo.position_x() - target_x);
        double y_err = Math.abs(drive.odo.position_y() - target_y);
        if (x_err < 5 && y_err < 5 ){
            return true;
        }
        return false;*/
    }

}
