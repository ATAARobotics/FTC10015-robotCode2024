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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Bench Test", group="Opmode")
public class BenchTest extends OpMode   {
    public Drive drive = null;
    public Arm arm = null;
    // ideally make a class for "plane launcher", but it's just one servo so :shrug:
    public ServoEx plane_launcher;
    double plane_countdown_start = -1; // > 0 if we've started the timer
    protected boolean plane_launched = false;

    public GamepadEx driver = null;
    public GamepadEx operator = null;

    double bench_drive_start = -1.0;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        drive = new Drive(hardwareMap, null, arm, true);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        plane_launcher = new SimpleServo(hardwareMap, "plane", 0, 180.0, AngleUnit.DEGREES);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        drive.imu.resetYaw();
        drive.start();
        arm.reset();
        plane_launched = false;
    }

    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        // order is important here
        // we do "normal stuff" first via humanInputs() calls
        // then we maybe-override things (for bench-test stuff)
        // then we call loop() to do "actual" updates

        drive.humanInputs(driver, time);
        arm.humanInputs(operator, time);

        if (driver.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            if (bench_drive_start < 0.0) {
                bench_drive_start = time;
            }
            double power = time - bench_drive_start;
            if (power > 1.0) {
                power = 1.0;
            }
            drive.motor_fl.set(power);
            drive.motor_fr.set(power);
            drive.motor_bl.set(power);
            drive.motor_br.set(power);
        } else {
            bench_drive_start = 0.0;
            drive.motor_fl.set(0.0);
            drive.motor_fr.set(0.0);
            drive.motor_bl.set(0.0);
            drive.motor_br.set(0.0);
        }

        // valid intake positions are:
        // intake_position = 1.0; // fully down
        // intake_position = 0.66; // resting
        // intake_position = 0.45; // stowed

        // for bench test we're doing triggers:
        // both down:stowed
        // left or right down onle: resting
        // both up: fully down
        if (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5 &&
            operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5) {
            // nothing is on
            arm.intake.intake_position = 1.0;
        } else if (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 || operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            if (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 && operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                // both triggers down
                arm.intake.intake_position = 0.25;
            } else {
                // only one trigger down
                arm.intake.intake_position = 0.55;
            }
        } else {
            
        }
        arm.intake.suck_mode = Intake.SuckMode.NOTHING;
        arm.roller_state = Arm.Roller.Off;

        // do actual updates
        arm.loop(time);
        drive.loop(time);

        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("wrist", arm.wristp);
        pack.put("arm_pos", arm.arm_main.getCurrentPosition());
        pack.put("intake_main", arm.intake.intake_main.getPosition());
        pack.put("intake_rev", arm.intake.intake_rev.getPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }
}
