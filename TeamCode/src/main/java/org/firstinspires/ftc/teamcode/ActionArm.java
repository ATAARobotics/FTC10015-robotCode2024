package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionArm extends ActionBase {

    String desired = "unknown";
    double started = -1.0;

    ActionArm(String state) { desired=state; }

    public boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0.0) {
            started = time;
            pack.put("action-arm", desired);
            if (desired == "intake") {
                arm.intake();
            } else if (desired == "resting") {
                arm.resting();
            } else if (desired == "scoring") {
                arm.scoring();
            } else if (desired == "open") {
                arm.open_claw();
            } else if (desired == "low-scoring") {
                arm.low_scoring();
            }
        }
        pack.put("action-arm-at", arm.arm_control.atSetPoint());
        pack.put("action-arm-set", arm.arm_main.getCurrentPosition());
        pack.put("action-arm-target", arm.arm_control.getSetPoint());
        return arm.arm_control.atSetPoint() || (time - started > 2.0);
    }

    public void draw_field(TelemetryPacket pack) {
    }
}
