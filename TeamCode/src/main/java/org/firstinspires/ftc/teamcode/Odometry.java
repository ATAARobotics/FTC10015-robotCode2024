package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry {
    MotorEx par = null;
    MotorEx perp = null;
    public double y_current; //in mm
    public double x_current; //in mm
    public double x_last;
    public double y_last;
    public final double ticks_to_mm = Math.PI * 48 /2000;
    public Odometry(HardwareMap hardware){
        y_current = 0;
        y_last = 0;
        x_current = 0;
        x_last = 0;
        par = new MotorEx(hardware,"par");
        //perp = new MotorEx(hardware, "perp");
    }
    public void update (double heading, double time ){
        double delta_y = par.getCurrentPosition() - y_last;
        y_last = par.getCurrentPosition();
        y_current = y_current + (delta_y * ticks_to_mm );
    }
}
