package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoMoveTo extends ActionBase {

    double target_east;
    double target_north;

    boolean done_east = false;
    boolean done_north = false;
    PIDController east_control = null;
    PIDController north_control = null;
    double end_time = -1;

    public AutoMoveTo(double east_west_mm, double north_south_mm, double end) {
        target_east = east_west_mm;
        target_north = north_south_mm;
        end_time = end;
        east_control = new PIDController(0.2, 0.0, 0.0);
        north_control = new PIDController(0.2, 0.0, 0.0);
    }

    // note to self: keep target in mm (not ticks)!
    boolean update(double time, Drive drive, Telemetry telemetry) {
        telemetry.addData("x", drive.odo.position_x());
        telemetry.addData("y", drive.odo.position_y());

        // positive x is robot-backwards
        // positive y is robot-left

        east_control.setSetPoint(target_east);
        north_control.setSetPoint(target_north);
        if (end_time > 0.0 && time > end_time){
            telemetry.addData("done","done");
            drive.robotInputs(0, 0);
            return true;
        }
        if (! done_east){
            double strafe = - east_control.calculate(drive.odo.position_y());
            if (strafe < -0.5){
                strafe = -0.5;
            } else if (strafe > 0.5){
                strafe = 0.5;
            }
            telemetry.addData("strafe", strafe);
            //            drive.robotInputs(strafe, 0);
            double ew_error = target_east - drive.odo.position_y();
            telemetry.addData("east_error", ew_error);
            if (ew_error <= 0.0){
                done_east = true;
            }
        } else if (! done_north && false){
            double forward = north_control.calculate(drive.odo.position_y());
            telemetry.addData("forward", forward);
            drive.robotInputs(0.0, forward);
            double ns_error = Math.abs(target_north - drive.odo.position_y());
            telemetry.addData("north_error", ns_error);
            if (ns_error < 5){
                done_north = true;
            }
        } else {
            return true;
        }
        return false;
    }

}
