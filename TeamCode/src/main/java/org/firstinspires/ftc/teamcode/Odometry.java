package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry {
    MotorEx par = null;
    MotorEx perp = null;
    protected double x_last; //in mm
    protected double y_last; //in mm
    protected boolean is_paused;
    public final double ticks_to_mm = Math.PI * 48 /2000;
    public Odometry(HardwareMap hardware){
        //when we "pause", record our last position ..
        // on resume we reset the encoders and then our new position is relative to this
        y_last = 0;
        x_last = 0;
        is_paused = false;
        par = new MotorEx(hardware,"par");
        par.resetEncoder();
        perp = new MotorEx(hardware, "perp");
        perp.resetEncoder();
    }
    public double position_y(){
        return y_last + par.getCurrentPosition();
    }
    public double position_x(){
        return x_last + perp.getCurrentPosition();
    }

    public void pause(double heading){
        y_last = position_y();
        x_last = position_x();
        is_paused = true;
    }

    public void resume(double heading){
        is_paused = false;
        //need to account for "heading" and possible swap/invert things (future)
        par.resetEncoder();
        perp.resetEncoder();
    }
}
