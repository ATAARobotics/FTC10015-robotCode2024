package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drive {
    public Odometry odo = null;
    public Motor motor_fl = null;
    public Motor motor_fr = null;
    public Motor motor_bl = null;
    public Motor motor_br = null;

    MecanumDrive drivebase = null;
    IMU imu;
    PIDController headingControl = null;

    // have to pretend our encoders are motors
    public Motor parallel_encoder = null;

    // inputs into the drivebase, from human or auto
    private double forward = 0.0;
    private double strafe = 0.0;
    private double turn = 0.0;
    private double heading = 0.0;  // from IMU

    public Drive(HardwareMap hardwareMap) {
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motor_fl = new Motor(hardwareMap, "FL_Drive");
        motor_fr = new Motor(hardwareMap, "FR_Drive");
        motor_bl = new Motor(hardwareMap, "BL_Drive");
        motor_br = new Motor(hardwareMap, "BR_Drive");

        // 48mm wheel, 2000 ticks-per-rev
        parallel_encoder = new Motor(hardwareMap, "par", 2000, 1.0);
        parallel_encoder.setDistancePerPulse((48.0 * Math.PI) / 2000.0);
        parallel_encoder.resetEncoder();

        // in "turbe" mode, 0.2 + 0.1 + 0.0 was oscilating a lot (but was good in non-turbo mode)
        headingControl = new PIDController(0.08, 0.05, 0.0);
        // using ftc-lib for driving
        drivebase = new MecanumDrive(motor_fl, motor_fr, motor_bl, motor_br);
        drivebase.setMaxSpeed(0.6);

        odo = new Odometry(hardwareMap);
    }

    public void start() {
        // called once per game
        imu.resetYaw();
        headingControl.setSetPoint(0.0);
    }

    public double getHeading() { return heading; }

    public void humanInputs(GamepadEx driver){
        // this method called ONCE per loop from teleop controller
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.DEGREES);
        double correction = headingControl.calculate(heading);

        // heading-lock from DPAD
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            headingControl.setSetPoint(0.0);
        } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            headingControl.setSetPoint(90.0);
        } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            headingControl.setSetPoint(-90.0);
        } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            headingControl.setSetPoint(0.0);
        }

        // turbo mode or not
        // triggers return 0.0 -> 1.0 "more than 0.5" is "more than half pressed"
        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            drivebase.setMaxSpeed(0.85);
            headingControl.setPID(0.03, 0.00, 0.001);
        } else {
            drivebase.setMaxSpeed(0.55);
            headingControl.setPID(0.05, 0.00, 0.002);
        }

        forward = -driver.getRightY();
        strafe = driver.getRightX();
        turn = -correction;
    }
    public void loop(double time) {
        /// also called once per loop, can do autonomous updates etc here

        // tell ftclib its inputs
        drivebase.driveFieldCentric(strafe, forward, turn, heading);
    }

}
