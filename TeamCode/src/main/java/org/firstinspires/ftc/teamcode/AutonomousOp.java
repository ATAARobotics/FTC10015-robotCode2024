package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.LinkedList;
import java.util.List;

//@Autonomous(name="Autonomous9000", group="Autonomous")
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AutoTele9000", group = "Opmode")
public class AutonomousOp extends OpMode {

    private Drive drive;
    private Intake intake;
    boolean intake_is_up = true;
    private Arm arm;

    private GamepadEx pad;

    // where we think the team element is
    private int position = -1;
    private LinkedList<ActionBase> actions;
    private ActionBase current_action;

    static final String[] LABELS = {"red", "blue"};


    // copied from ConceptTensorFlowObjectDetection example
    private TfodProcessor tfod;
    VisionPortal visionPortal;


    /*
    notes:
    - want to do a series of actions
    - encoded as a list, maybe?
    - first: wait until Tfod detects the team element / pixel (figure out "position")
    - drive to board
    - put pixel on board (yellow)
    - drive to "correct position", spit out pixel (purple)
    - drive to stack, suck in two pixels
    - drive to board, put pixels on
    - repeat above if enough time remains (i.e "if >5 seconds left, go get more"?)


    each action will want an "update" method:
      - giving it a chance to give commands to Drive or Arm
      - update its own state
    each action needs a way to signal "I am done"
    overall control loop looks at:
      - are we "nearly out of time"? (yes: drive to board)
      - do we have a current action? (no: pop one from list)
      - is the current action done? (yes: pop a new one from list)

    all the "drive to" actions should use a single lane -- so they should be a "two part"
    action that does the horizontal motion first, then the north/south motion (then any remaining east/west?)
     */

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        pad = new GamepadEx(gamepad1);

        actions = new LinkedList<ActionBase>();

        if (true) {
            actions.add(new ActionMove(0, 300));
            actions.add(new ActionNothing());
        }

        if (false) {
            // from start position: 17 inches forward, 6.5 inches right (position 1)
            // positive y is robot-forward, positive x is robot-right
            actions.add(new ActionMove(165, 401));
            actions.add(new ActionIntake(false));
            actions.add(new ActionSuck(false));
            actions.add(new ActionArm("resting"));
            actions.add(new ActionIntake(true));
            actions.add(new ActionTurn(90));
            actions.add(new ActionMove(165, 431 + 241));
            actions.add(new ActionMove(165 - 2260, 431 + 241));
            actions.add(new ActionArm("scoring"));
            actions.add(new ActionArm("open"));
            actions.add(new ActionArm("resting"));
        }
        if (false) {
            // from start position: 17 inches forward, 6.5 inches right (position 1)
            // positive y is robot-forward, positive x is robot-right
            actions.add(new ActionMove(165, 380));
            actions.add(new ActionIntake(false));
            actions.add(new ActionSuck(false));
            actions.add(new ActionArm("resting"));
            actions.add(new ActionTurn(-90));
            actions.add(new ActionMove(265, 380));
            actions.add(new ActionMove(265, 411 + 460));
            actions.add(new ActionMove(165 - 2260 + 444, 411 + 460));
            actions.add(new ActionMove(165 - 2260 + 444, 411 + 66));
            actions.add(new ActionArm("resting"));
            actions.add(new ActionArm("low-scoring"));
            actions.add(new ActionArm("open"));
            actions.add(new ActionArm("intake"));
            actions.add(new ActionIntake(false));
        }

        // move to center lane, then to go board
        if (false) {
            actions.add(new ActionMove(0, 666));
            actions.add(new ActionMove(-1200, 666));
        }

        // vision (from the example code)
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //////.setModelFileName("model_20231118_125258.tflite")
                .setModelFileName("model_20231118_143732.tflite")
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "cam_1"));
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
        time = 0.0;
        drive.start();
        drive.imu.resetYaw();
        intake_is_up = true;
        //FtcDashboard.getInstance().startCameraStream(camera, 0);
    }

    @Override
    public void loop() {
        if (true) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());
            telemetry.addData("time", time);
            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }   // end for() loop

            telemetry.update();
        }

        TelemetryPacket pack = new TelemetryPacket();

        if (current_action != null) {
            boolean rtn = current_action.update(time, drive, intake, arm, telemetry, pack);
            if (rtn) {
                current_action = null;
            }
        }
        if (current_action == null) {
            if (actions.size() > 0) {
                current_action = actions.removeFirst();
            }
        }

        // directions (in start position):
        // +x = robot-right
        // -x = robot-left
        // +y = robot forward
        // -y = robot backwards

        // rotations:
        // +90 == turn left (??)
        // -90 == turn right

        if (current_action == null) {
            // we aren't doing anything, and have nothing else to do -- accept input from controllers
            pad.readButtons();
            pack.put("DPAD_UP", pad.wasJustPressed(GamepadKeys.Button.DPAD_UP));
            if (pad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                current_action = new ActionMove(drive.odo.position_x() - 333, drive.odo.position_y());
            } else if (pad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                current_action = new ActionMove(drive.odo.position_x() + 333, drive.odo.position_y());
            } else if (pad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                current_action = new ActionMove(drive.odo.position_x(), drive.odo.position_y() - 333);
            } else if (pad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                current_action = new ActionMove(drive.odo.position_x(), drive.odo.position_y() + 333);
            } else if (pad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                double heading = drive.headingControl.getSetPoint();
                heading -= 90;
                if (heading < -90.0) {
                    heading = 180;

                }
                current_action = new ActionTurn(heading);
            } else if (pad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                double heading = drive.headingControl.getSetPoint();
                heading += 90;
                if (heading > 180) {
                    heading = -90;

                }
                current_action = new ActionTurn(heading);
            } else if (pad.wasJustPressed(GamepadKeys.Button.A)) {
                if (intake_is_up) {
                    current_action = new ActionIntake(false);
                } else {
                    current_action = new ActionIntake(true);
                }
                intake_is_up = !intake_is_up;
            } else if (pad.wasJustPressed(GamepadKeys.Button.B)) {
                current_action = new ActionSuck(false);
            }
        }

        // seems to not work if "data" and "drawing" are in the same packet?
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        pack = new TelemetryPacket();

        // actual robot is 407mm square
        double INCHES_TO_MM = 0.03937008;

        // origin to start position (XXX overridable method for this?)
        pack.fieldOverlay().setTranslation(-6 * 12, 4 * 12);

        // do all other drawing in millimeters
        pack.fieldOverlay().setScale(INCHES_TO_MM, INCHES_TO_MM);
        pack.fieldOverlay().setRotation(-Math.toRadians(90));
        // center the drawing in the robot
        //pack.fieldOverlay().setTranslation(-203, 203);
        pack.fieldOverlay()
                .setFill("green")
                .fillCircle(0.0, 0.0, 2.0)
                .setFill("red")
                .fillRect(drive.odo.position_y() - (407 / 2), drive.odo.position_x() - (407 / 2), 407, 407);

        if (current_action != null) {
            current_action.draw_field(pack);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        drive.loop(time);
        arm.loop(time);
        intake.loop(time);
        telemetry.update();
    }
}
