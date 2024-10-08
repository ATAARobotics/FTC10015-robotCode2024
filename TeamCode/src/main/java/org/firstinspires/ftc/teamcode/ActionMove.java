package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionMove extends ActionBase {

    double target_x;
    double target_y;
    double started = -1;
    boolean slow = false;
    double timeout = 5.0;
    PIDController control_x;
    PIDController control_y;


    ActionMove(double x, double y) {
        timeout = 5.0;
        control_x = new PIDController(0.02, 0.15,0.0025);
        control_y = new PIDController(0.02, 0.15, 0.0025);
      //  control_x = new PIDController(0.005, 0.08,0.0025);
      //  control_y = new PIDController(0.005, 0.08, 0.0025);
        // from the april-tag locker .. also set a max-speed!
        //control_x = new PIDController(0.01, 0.05, 0.0005);
        //control_y = new PIDController(0.01, 0.05, 0.0005);
        control_x.setTolerance(5);
        control_y.setTolerance(5);
        set_target(x, y);
    }

    ActionMove(double x, double y, double timeout) {
        this.timeout = timeout;
        // thinking:
        // at 100mm away, start decreasing speed
        // error is "100" when we're 100mm away
        // so since 100 * 0.001 == 1.0 set Kp to 0.001
        //
        // using GM0.org article, we tune from there:
        // increase Kp until there are oscillations
        // (old tunings: 0.015, 0.0002, 0)
        control_x = new PIDController(0.02, 0.15,0.0025);
        control_y = new PIDController(0.02, 0.15, 0.0025);
      //  control_x = new PIDController(0.005, 0.08,0.0025);
      //  control_y = new PIDController(0.005, 0.08, 0.0025);
        // from the april-tag locker .. also set a max-speed!
        //control_x = new PIDController(0.01, 0.05, 0.0005);
        //control_y = new PIDController(0.01, 0.05, 0.0005);
        control_x.setTolerance(5);
        control_y.setTolerance(5);
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

        if (true) {
            double speedcap = 0.75;
            if (!slow) { speedcap = 0.95; }
            if (forward > speedcap) { forward = speedcap; }
            if (forward < -speedcap) { forward = -speedcap; }
            if (strafe > speedcap) { strafe = speedcap; }
            if (strafe < -speedcap) { strafe = -speedcap; }
        }

        if (started < 0.0) {
            started = time;
        }
        telemetry.addData("strafe", strafe);
        telemetry.addData("forward", forward);
        telemetry.addData("target_x", target_x);
        telemetry.addData("target_y", target_y);
        telemetry.addData("at_target", at_target(drive));

        pack.put("strafe", strafe);
        pack.put("forward", forward);
        pack.put("target_x", target_x);
        pack.put("target_y", target_y);
        pack.put("at_target", at_target(drive));

        if (at_target(drive) || timed_out(time)) {
            drive.robotInputs(0, 0);
            return true;
        }
        drive.robotInputs(strafe, forward);
        return false;
    }

    private boolean timed_out(double time) {
        if (started > 0) {
            return (time - started) > timeout;
        }
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
