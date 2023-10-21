package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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


@TeleOp(name="DriveMaster9000", group="Opmode")
public class TeleOpTest extends OpMode   {

    public Motor motor_fl = null;
    public Motor motor_fr = null;
    public Motor motor_bl = null;
    public Motor motor_br = null;

    public GamepadEx driver = null;

    MecanumDrive drivebase = null;
    IMU imu;
    PIDController headingControl = null;

    // copied from ConceptTensorFlowObjectDetection example
    private TfodProcessor tfod;
    VisionPortal visionPortal;
    double target_heading = 0;
    double correction = 0;



    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        driver = new GamepadEx(gamepad1);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motor_fl = new Motor(hardwareMap, "FL_Drive");
        motor_fr = new Motor(hardwareMap, "FR_Drive");
        motor_bl = new Motor(hardwareMap, "BL_Drive");
        motor_br = new Motor(hardwareMap, "BR_Drive");

        // in "turbe" mode, 0.2 + 0.1 + 0.0 was oscilating a lot (but was good in non-turbo mode)
        headingControl = new PIDController(0.08, 0.05, 0.0);
        // using ftc-lib for driving
        drivebase = new MecanumDrive(motor_fl, motor_fr, motor_bl, motor_br);
        drivebase.setMaxSpeed(0.6);

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
        imu.resetYaw();
        headingControl.setSetPoint(0.0);
    }


    @Override
    public void loop() {
        driver.readButtons();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        //correction = target_heading - heading;
        correction = headingControl.calculate(heading);

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            headingControl.setSetPoint(0.0);
        } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            headingControl.setSetPoint(90.0);
        } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            headingControl.setSetPoint(-90.0);
        } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            headingControl.setSetPoint(0.0);
        }

        // triggers return 0.0 -> 1.0 "more than 0.5" is "more than half pressed"
        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            drivebase.setMaxSpeed(0.85);
        } else {
            drivebase.setMaxSpeed(0.55);
        }
        // tell ftclib its inputs
        drivebase.driveFieldCentric(
                driver.getRightX(),
                -driver.getRightY(),
               /// driver.getLeftX(),
                -correction , //gamepad1.left_stick_x,
                heading
        );
        //imu stuff
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", heading);
        telemetry.addData("target","%2f target heading", target_heading);
        //telemetryTfod();
        telemetry.update();
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
