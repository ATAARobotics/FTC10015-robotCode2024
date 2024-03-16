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
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Drive {
    public Odometry odo = null;
    public Motor motor_fl = null;
    public Motor motor_fr = null;
    public Motor motor_bl = null;
    public Motor motor_br = null;

    MecanumDrive drivebase = null;
    IMU imu;
    PIDController headingControl = null;

    private Arm arm; // ONLY for looking at arm-position for april-lock

    // have to pretend our encoders are motors
   // public Motor parallel_encoder = null;

    // inputs into the drivebase, from human or auto
    public double forward = 0.0;
    private double strafe = 0.0;
    private double turn = 0.0;
    private double heading = 0.0;  // from IMU

    AprilLock april_locker;
    int last_april_tag = 1;  // used by april-tag locker

    public Drive(HardwareMap hardwareMap, AprilTagPipeline pipe, Arm a, boolean is_red) {
        arm = a;
        april_locker = new AprilLock(pipe, is_red);

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
        //parallel_encoder = new Motor(hardwareMap, "par", 2000, 1.0);
        //parallel_encoder.setDistancePerPulse((48.0 * Math.PI) / 2000.0);
        //parallel_encoder.resetEncoder();

        // in "turbo" mode, 0.2 + 0.1 + 0.0 was oscilating a lot (but was good in non-turbo mode)
        headingControl = new PIDController(0.08, 0.05, 0.0);
        // using ftc-lib for driving
        drivebase = new MecanumDrive(motor_fl, motor_fr, motor_bl, motor_br);
        drivebase.setMaxSpeed(0.6);

        odo = new Odometry(hardwareMap);
    }

    public void start() {
        this.start(0.0);
    }

    public void start(double heading) {
        headingControl.setSetPoint(heading);
    }

    public double getHeading() { return heading; }

    public void robotInputs(double d_strafe, double d_forward){
        strafe = d_strafe;
        forward = d_forward;
        headingControl.setPID(0.03, 0.0, 0.001);
        headingLock();
    }

    public void humanInputs(GamepadEx driver, double time){
        // this method called ONCE per loop from teleop controller

        // tweak "left" or "right" pixel based on last dpad press
        // (but, yes let them do this any time)
        if (false) {
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                april_locker.control_x.setSetPoint(0.0);
            } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                april_locker.control_x.setSetPoint(-10.0);
            }
        }

        // if X, A, B is held down, we only look at an April tag (otherwise,
        // we let the other controls work)
        if (driver.isDown(GamepadKeys.Button.X) || driver.isDown(GamepadKeys.Button.A) || driver.isDown(GamepadKeys.Button.B)) {

            // could we do something like "move up to 6cm left/right" on dpad press?
            if (driver.wasJustPressed(GamepadKeys.Button.X) || driver.wasJustPressed(GamepadKeys.Button.A) || driver.wasJustPressed(GamepadKeys.Button.B)) {
                if (arm.state == Arm.Position.MediumScoring || driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                    april_locker.close_position();
                } else {
                    april_locker.far_position();
                }
            }
            last_april_tag = 1; // or 4
            if (driver.isDown(GamepadKeys.Button.A)) {
                last_april_tag = 2; // or 5
            } else if (driver.isDown(GamepadKeys.Button.B)) {
                last_april_tag = 3; // or 6
            }
            aprilLock(time, last_april_tag);

            // TEMPORARY for tuning the locker PIDs
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                april_locker.control_y.setD(april_locker.control_y.getD() * 1.1);
            } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                april_locker.control_y.setD(april_locker.control_y.getD() * 0.9);
            }
            // positive / negative here depends on red/blue alliance? this for red
            forward = -april_locker.fwd;
            strafe = april_locker.strafe;
        } else {
            // XXX FIXME need to stop april-tag locker?
            //april_locker.started = -1;
            // heading-lock from right joystick
            // (HAVE TO FIX for "backwards" autonomous start)

            if (driver.getLeftX() < -0.5) {
                headingControl.setSetPoint(-90.0); // west
            } else if (driver.getLeftX() > 0.5) {
                headingControl.setSetPoint(90.0); // east
            } else if (driver.getLeftY() < -0.5) {
                headingControl.setSetPoint(0); // south
            } else if (driver.getLeftY() > 0.5) {
                headingControl.setSetPoint(180.0); // north
            }

            // turbo mode or not
            // triggers return 0.0 -> 1.0 "more than 0.5" is "more than half pressed"
            if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                drivebase.setMaxSpeed(1.0);
                headingControl.setPID(0.03, 0.00, 0.001);
            } else {
                drivebase.setMaxSpeed(0.50);
                headingControl.setPID(0.05, 0.00, 0.002);
            }

            // when we started "forward" this was true:
//            forward = -driver.getRightY();
//            strafe = driver.getRightX();
            // ...but now we usually start "backwards"
            forward = driver.getRightY();
            strafe = -driver.getRightX();

            // virtual fence
            if (false) {
                double dist = april_locker.pipeline.closestAprilTag();
                // XXX depends on red/blue ... this for red
                if (dist > 0.0 && dist < 300 && strafe < 0) {
                    strafe = 0;
                }
            }
        }
        headingLock();
    }

    // lock on to a particular April tag
    private void aprilLock(double time, int tag_id) {
        april_locker.pipeline.set_target(tag_id);
        april_locker.update(time);
    }

    private void headingLock() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.DEGREES);

        if (headingControl.getSetPoint() == 180.0) {
            // "south" is special because it's around the 180/-180 toggle-point
            double h = heading;
            // okay so if imu heading is actually -175 then we want to pretend
            // the heading is 185 (for example) to get right "correction" out of the
            // PID controller
            if (h < 0.0) {
                h = 180 + (heading + 180);
            }
            double correction = headingControl.calculate(h);
            turn = -correction;
        } else {
            double correction = headingControl.calculate(heading);
            turn = -correction;
        }
    }
    public void loop(double time) {
        /// also called once per loop, can do autonomous updates etc here

        // tell ftclib its inputs
        drivebase.driveFieldCentric(strafe, forward, turn, heading);

        // if we want tank-drive for some reason:
        //drivebase.driveRobotCentric(strafe, forward, turn);
    }

}
