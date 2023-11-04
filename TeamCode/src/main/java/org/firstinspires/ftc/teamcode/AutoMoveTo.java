package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

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
    }

    boolean update(double time, Drive drive) {
        if (end_time > 0.0 && time > end_time){
            drive.robotInputs(0, 0);
            return true;
        }
        if (! done_east){
            drive.robotInputs(east_control.calculate(drive.odo.position_x()), 0);
            double ew_error = Math.abs(target_east - drive.odo.position_x());
            if (ew_error < 5){
                done_east = true;
            }
        } else if (! done_north){
            drive.robotInputs(0.0, north_control.calculate(drive.odo.position_y()));
            double ns_error = Math.abs(target_north - drive.odo.position_y());
            if (ns_error < 5){
                done_north = true;
            }
        } else {
            return true;
        }
        return false;
    }

}
