package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// put all FTCLib imports here
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="DriveMaster9000", group="Opmode")
public class TeleOp extends OpMode   {
    public Drive drive = null;
    public Arm arm = null;
    // ideally make a class for "plane launcher", but it's just one servo so :shrug:
    public ServoEx plane_launcher;
    double plane_countdown_start = -1; // > 0 if we've started the timer
    boolean plane_launched = false;

    public GamepadEx driver = null;
    public GamepadEx operator = null;

    protected OpenCvWebcam megacam;
    protected AprilTagPipeline pipeline;

    BNO055IMU arm_imu = null;

    @Override
    public void init() {
        pipeline = new AprilTagPipeline(1);
        megacam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "megacam"));
        megacam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                megacam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE);
                megacam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        arm = new Arm(hardwareMap);
        drive = new Drive(hardwareMap, pipeline, arm);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        plane_launcher = new SimpleServo(hardwareMap, "plane", 0, 180.0, AngleUnit.DEGREES);

        //arm_imu = hardwareMap.get(BNO055IMU.class, "arm imu");
        //arm_imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void init_loop() {
        telemetry.addData("april", pipeline.has_result());
        telemetry.addData("distance", pipeline.distance());
    }

    @Override
    public void start() {
        drive.imu.resetYaw();
        drive.start();
        arm.arm.resetEncoder();
        plane_launched = false;
    }

    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        drive.humanInputs(driver, time);
        drive.loop(time);
        arm.humanInputs(operator);
        arm.loop(time);
        // ideally put in PlaneLauncher or something, but for now it lives here
        // (if we press Y for more than 0.6 seconds, release the plane)
        if (operator.isDown(GamepadKeys.Button.Y)) {
            if (plane_countdown_start < 0) {
                plane_countdown_start = time;
            }
            double elapsed = time - plane_countdown_start;
            if (elapsed > 0.6) {
                plane_launcher.setPosition(1.0);
                plane_launched = true;
            }
        } else {
            plane_countdown_start = -1;
        }

        // if we already launched the plane, pressing Y shuts the trigger again
        if (plane_launched && operator.wasJustPressed(GamepadKeys.Button.Y)){
            plane_launcher.setPosition(0.5);
            plane_launched = false;
            plane_countdown_start = -1;
        }

        //imu stuff
        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();
      //  pack.put("arm_yaw", arm_imu.getAngularOrientation());
        pack.put("pos_y", drive.odo.position_y());
        pack.put("pos_x", drive.odo.position_x());
        pack.put("heading", drive.getHeading());
        pack.put("target_heading", drive.headingControl.getSetPoint());
        pack.put("arm_pos", arm.arm_main.getCurrentPosition());
        pack.put("intake_angle", arm.intake.intake_main.getAngle());
        pack.put("claw", arm.clawp);
        pack.put("wrist", arm.wristp);
        pack.put("close_april", drive.april_locker.pipeline.closestAprilTag());
        pack.put("april_lock", drive.april_locker.locked());
        pack.put("april_target", drive.last_april_tag);
        pack.put("april_tag_target", drive.april_locker.pipeline.has_result());
        pack.put("april_tag_distance", drive.april_locker.pipeline.distance());
        pack.put("april_fwd", drive.forward);
        pack.put("april_y_p", drive.april_locker.control_y.getP());
        pack.put("april_y_i", drive.april_locker.control_y.getI());
        pack.put("april_y_d", drive.april_locker.control_y.getD());
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

}
