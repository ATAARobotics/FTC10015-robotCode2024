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
    public Arm arm = null;
    public GamepadEx driver = null;
    public GamepadEx operator = null;

    BNO055IMU arm_imu = null;

    @Override
    public void init() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        arm = new Arm(hardwareMap);

        //arm_imu = hardwareMap.get(BNO055IMU.class, "arm imu");
        //arm_imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void start() {
        drive.start();
    }

    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        drive.humanInputs(driver);
        drive.loop(time);
        arm.update(operator);

        //imu stuff
        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();
      //  pack.put("arm_yaw", arm_imu.getAngularOrientation());
        pack.put("pos_y", drive.odo.position_y());
        pack.put("pos_x", drive.odo.position_x());
        pack.put("heading", drive.getHeading());
        pack.put("target_heading", drive.headingControl.getSetPoint());
        pack.put("arm_pos", arm.arm_main.get());
        pack.put("intake_angle", arm.intake.intake_main.getAngle());
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
