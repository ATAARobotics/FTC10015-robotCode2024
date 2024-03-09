package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionArm extends ActionBase {

    String desired = "unknown";
    double started = -1.0;
    double max_time = 2.0;

    ActionArm(String state) {
        desired = state;
        max_time = 2.0;
    }
    ActionArm(String state, double timeout) {
        desired = state;
        max_time = timeout;
    }

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0.0) {
            started = time;
            // TODO FIXME this needs to be Enum, not strings!
            pack.put("action-arm", desired);
            if (desired == "intake") {
                arm.intake();
            } else if (desired == "resting") {
                arm.resting();
            } else if (desired == "scoring") {
                arm.high_scoring();
            } else if (desired == "spit-out") {
                arm.roller_out();
            } else if (desired == "spit-stop") {
                arm.roller_off();
            } else if (desired == "low-scoring") {
                arm.low_scoring();
            } else if (desired == "close") {
                //arm.close_claw();
            }
        }
        pack.put("action-arm-at", arm.arm_control.atSetPoint());
        pack.put("action-arm-set", arm.arm_main.getCurrentPosition());
        pack.put("action-arm-target", arm.arm_control.getSetPoint());
        //return arm.arm_control.atSetPoint() || (time - started > 2.0);
        return (time - started) > max_time;
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
