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

    OpenCvWebcam camera;
    AprilTagPipeline pipeline;
    double started = -1;
    double last_result = 0.0;
    PIDController control_x;
    PIDController control_y;


    ActionAprilLock(OpenCvWebcam cm, int tag_id) {
        camera = cm;
        pipeline = new AprilTagPipeline(tag_id);
        pipeline.setDecimation(3); // "HIGH" from example https://github.com/OpenFTC/EOCV-AprilTag-Plugin/blob/master/examples/src/main/java/org/firstinspires/ftc/teamcode/AprilTagDemo.java
        control_x = new PIDController(0.005, 0.005, 0.0005);
        control_y = new PIDController(0.005, 0.005, 0.0005);
        control_x.setTolerance(5);
        control_y.setTolerance(5);
        control_x.setSetPoint(0);
        control_y.setSetPoint(500); // mm
    }
    boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0) {
            started = time;
            camera.setPipeline(pipeline);
        }

        pack.put("has-target", pipeline.has_result());
        pack.put("last-result", (time - last_result));
        if (pipeline.has_result()) {
            last_result = time;
            double fwd = control_y.calculate(pipeline.distance());
            double strafe = control_x.calculate(pipeline.strafe());
            if (false) {
                // "simple static-friction feed-forward"
                // (if we're trying to move "at all", make it at least 0.1 input)
                if (fwd < 0.01) {
                    fwd = -0.10 + fwd;
                } else if (fwd > 0.01) {
                    fwd = 0.10 + fwd;
                }
            }
            // cap max speed at 0.5 -- we're in "delicate" zone
            if (fwd > 0.5) { fwd = 0.5; }
            if (fwd < -0.5) { fwd = -0.5; }
            if (strafe > 0.5) { strafe = 0.5; }
            if (strafe < -0.5) { strafe = -0.5; }
            pack.put("pipe-strafe", pipeline.strafe());
            pack.put("pipe-distance", pipeline.distance());
            pack.put("stick-fwd", fwd);
            pack.put("stick-strafe", strafe);
            drive.robotInputs(
                strafe,
                fwd
            );
        } else {
            drive.robotInputs(0, 0);
            control_x.setSetPoint(0);
            control_y.setSetPoint(500);
        }
        return control_x.atSetPoint() && control_y.atSetPoint();
    }

    void draw_field(TelemetryPacket pack){

    }
}
