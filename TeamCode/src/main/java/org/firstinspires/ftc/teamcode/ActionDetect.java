package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.RedCubePipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class ActionDetect extends ActionBase {

    OpenCvWebcam camera;
    RedCubePipeline pipeline;
    double started = -1;
    String result = "unknown";
    ActionDetect(HardwareMap hardwareMap) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "cam+1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        pipeline = new RedCubePipeline();
    }
    boolean update(double time, Drive drive, Intake intake, Arm arm, Telemetry telemetry, TelemetryPacket pack) {
        if (started < 0) {
            started = time;
            camera.openCameraDevice();
            camera.startStreaming(640, 480);
            camera.setPipeline(pipeline);
        }
        if (pipeline.result != "unknown") {
            result = pipeline.result;
        }
        return (started + 2.0) < time;
    }

    void draw_field(TelemetryPacket pack){

    }
}
