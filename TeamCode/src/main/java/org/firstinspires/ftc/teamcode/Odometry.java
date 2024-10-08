package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry {
    MotorEx par = null;
    MotorEx perp = null;

    enum Heading {F, L, R, B;} // how to swap odos; F=forward, etc relative to start
    protected double x_last; //in mm
    protected double y_last; //in mm
    protected boolean is_paused;
    protected Heading heading = Heading.F;
    public final double ticks_to_mm = Math.PI * 48 /2000;
    public Odometry(HardwareMap hardware){
        //when we "pause", record our last position ..
        // on resume we reset the encoders and then our new position is relative to this
        y_last = 0;
        x_last = 0;
        is_paused = false;
        par = new MotorEx(hardware,"par");
        par.resetEncoder();
        perp = new MotorEx(hardware, "suck"); // reusing intake encoder
        perp.resetEncoder();
    }
    public double position_y(){
        double y = 0.0;
        switch (heading) {
            case F:
                y = y_last - (perp.getCurrentPosition() * ticks_to_mm);
                break;
            case B:
                y = y_last + (perp.getCurrentPosition() * ticks_to_mm);
                break;
            case R:
                y = y_last + (par.getCurrentPosition() * ticks_to_mm);
                break;
            case L:
                y = y_last - (par.getCurrentPosition() * ticks_to_mm);
                break;
        }
        return y;
    }
    public double position_x(){
        double x = 0.0;
        switch (heading) {
            case F:
                x = x_last - (par.getCurrentPosition() * ticks_to_mm);
                break;
            case B:
                x = x_last + (par.getCurrentPosition() * ticks_to_mm);
                break;
            case R:
                x = x_last - (perp.getCurrentPosition() * ticks_to_mm);
                break;
            case L:
                x = x_last + (perp.getCurrentPosition() * ticks_to_mm);
                break;
        }
        return x;
    }

    public void pause(){
        if (!is_paused) {
            y_last = position_y();
            x_last = position_x();
            is_paused = true;
        }
    }

    public void resume(double head){
        if (is_paused) {
            is_paused = false;
            par.resetEncoder();
            perp.resetEncoder();
            if (head == 0.0) {
                heading = Heading.F;
            } else if (head == -90) {
                heading = Heading.R;
            } else if (head == -180 || head == 180) {
                heading = Heading.B;
            } else if (head == 90) {
                heading = Heading.L;
            }
        }
    }
}
