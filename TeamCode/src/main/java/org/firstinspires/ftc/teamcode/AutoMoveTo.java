package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class AutoMoveTo extends ActionBase {

    double target_east;
    double target_north;

    boolean done_east = false;
    boolean done_north = false;
    PIDController east_control = null;
    PIDController north_control = null;

    public AutoMoveTo(double east_west_mm, double north_south_mm) {
        target_east = east_west_mm;
        target_north = north_south_mm;
    }

    boolean update(double time, Drive drive) {
        return false;
    }

}
