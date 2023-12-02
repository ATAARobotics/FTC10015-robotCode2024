package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class AprilLock {
    OpenCvWebcam camera;
    public AprilTagPipeline pipeline;
    public double started = -1;
    public double last_result = 0.0;
    PIDController control_x;
    PIDController control_y;

    public double strafe = 0;
    public double fwd = 0;

    AprilLock(OpenCvWebcam cm, int tag_id) {
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

    // calculate (strafe, forward) values for control (would be nice
    // to return both but java makes that awkward, so we use .fwd and
    // .strafe)
    void update(double time) {
        if (started < 0) {
            started = time;
            camera.setPipeline(pipeline);
        }

        if (pipeline.has_result()) {
            last_result = time;
            fwd = control_y.calculate(pipeline.distance());
            strafe = control_x.calculate(pipeline.strafe());
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
        }
        else
        {
            // lost lock; reset the setpoints so that the controllers
            // reset their I value history
            control_x.setSetPoint(0);
            control_y.setSetPoint(500);
            fwd = 0.0;
            strafe = 0.0;
        }
    }

    boolean locked() {
        return control_x.atSetPoint() && control_y.atSetPoint();
    }
}
