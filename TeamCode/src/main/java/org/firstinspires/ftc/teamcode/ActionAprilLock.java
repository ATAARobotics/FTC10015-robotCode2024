package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class ActionAprilLock extends ActionBase {

    AprilLock april;
    double started = -1;

    ActionAprilLock(OpenCvWebcam cm, int tag_id) {
        april = new AprilLock(cm, tag_id);
    }

    boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0) {
            started = time;
        }
        if (time - started > 2.5) {
            drive.robotInputs(0, 0);
            return true;
        }
        april.update(time);
        drive.robotInputs(april.strafe, april.fwd);

        pack.put("has-target", april.pipeline.has_result());
        pack.put("last-result", (time - april.last_result));
        pack.put("stick-fwd", april.fwd);
        pack.put("stick-strafe", april.strafe);
        return april.locked();
    }

    void draw_field(TelemetryPacket pack){

    }
}
