package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

// put all FTCLib imports here
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="DriveMaster9000", group="Opmode")
public class TeleOp extends OpMode   {
    public Drive drive = null;

    public GamepadEx driver = null;
    public GamepadEx operator = null;

    BNO055IMU arm_imu = null;


    // copied from ConceptTensorFlowObjectDetection example
    private TfodProcessor tfod;
    VisionPortal visionPortal;


    @Override
    public void init() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        //arm_imu = hardwareMap.get(BNO055IMU.class, "arm imu");
        //arm_imu.initialize(new IMU.Parameters(orientationOnRobot));

        //init_vision();
    }

    public void init_vision() {


        // vision (from the example code)
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        builder.setCameraResolution(new Size(640, 480));
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);
    }

    @Override
    public void start() {
        drive.start();
    }

    @Override
    public void loop() {
        driver.readButtons();

        drive.humanInputs(driver);
        drive.loop(time);

        //imu stuff
        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("arm_yaw", arm_imu.getAngularOrientation());
        pack.put("pos_y", drive.odo.position_y());
        pack.put("pos_x", drive.odo.position_x());
        pack.put("heading", drive.getHeading());
        pack.put("target_heading", drive.headingControl.getSetPoint());
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        // it seems that you can't send both "number" telemetry _and_ "draw stuff" telemetry in the same "packet"?
        pack = new TelemetryPacket();

        // actual robot is 407mm square
        double INCHES_TO_MM = 0.03937008;
        // move origin to bottom left
        pack.fieldOverlay().setTranslation(-6*12, 6*12);
        // do all other drawing in millimeters
        pack.fieldOverlay().setScale(INCHES_TO_MM, INCHES_TO_MM);
        // center the drawing in the robot
        //pack.fieldOverlay().setTranslation(-203, 203);
        pack.fieldOverlay()
 //               .setFill("blue")
  //              .fillCircle(parallel_encoder.getDistance(), 0.0, 2.0)
                .setFill("red")
                .fillRect(drive.odo.position_y() - (407/2), drive.odo.position_x() - (407/2), 407, 407);

        //telemetryTfod();
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        //telemetry.update();
    }

    private double mm_to_inches(double mm) {
        return mm * 0.03937008;
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
}
