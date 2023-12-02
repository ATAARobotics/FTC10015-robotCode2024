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
import org.firstinspires.ftc.teamcode.vision.ReverseTeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.TeamElementPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.LinkedList;
import java.util.List;

public abstract class AutonomousOp extends OpMode {

    private Drive drive;
    private Intake intake;
    boolean intake_is_up = true;
    private Arm arm;

    // where we think the team element is
    private LinkedList<ActionBase> actions;
    private ActionBase current_action;
    private ActionBase special_action;

    //OpenCvPipeline pipeline;
    ReverseTeamElementPipeline pipeline;
    OpenCvWebcam front_cam;
    OpenCvWebcam rear_cam;
    int target = -1;
    double last_loop;

    public enum Zone {NEAR, FAR};
    public enum Alliance {RED, BLUE};

    public abstract Zone getZone();
    public abstract Alliance getAlliance();

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
        actions = new LinkedList<ActionBase>();

        pipeline = new ReverseTeamElementPipeline(getAlliance() == Alliance.RED);
        front_cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam_1"));

        front_cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                front_cam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        rear_cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam_2"));
        rear_cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                rear_cam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE);
                rear_cam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        drive = new Drive(hardwareMap, rear_cam);
        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
    }

    @Override
    public void start() {
        resetRuntime();
        drive.start();
        drive.imu.resetYaw();
        intake_is_up = true;
        last_loop = time;
        special_action = null;//new ActionInitialIntake();
        arm.arm.resetEncoder();
        //FtcDashboard.getInstance().startCameraStream(camera, 0);
    }

    protected void createActions() {

        double TILE = 610; // 24inches = 610mm

        // We can call getZone() or getAlliance() to figure out which place we started in
        // ...also, "target" is valid when we get here (will be 1, 2 or 3).

        // negative y is robot-forward, negative x is robot-right

        actions.add(new ActionArm("close"));

        if (getZone() == Zone.FAR && getAlliance() == Alliance.BLUE) {
            if (target == 3) {
                // blue-side initial bits of motion .. spit out purple pixel
                // zone 3
                actions.add(new ActionMove(-385, -(TILE + TILE)));
                actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                // "common" tile? 3 out -- just past the "scoring" tile
                actions.add(new ActionMove(-150, -(TILE*2 + (165/2))));
            } else if (target == 2) {
                // blue-side initial bits of motion .. spit out purple pixel
                // zone 2
                actions.add(new ActionMove((-165/2), -(TILE  + 730)));
                actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(-150, -(TILE*2 + (165/2))));
            } else {
                // blue-side initial bits of motion .. spit out purple pixel
                // zone 1
                actions.add(new ActionMove(-165/2, -TILE));
                actions.add(new ActionTurn(-90));
                actions.add(new ActionMove(0, -TILE));
                actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(-150, -(TILE*2 + (165/2))));
            }

            // no matter what we did with the pixel above, we're in the same position and can go to the board

            if (true) {
                actions.add(new ActionTurn(90)); //turn to face the arm towards the backdrop
                actions.add(new ActionMove(((3 * TILE) + (165 / 2)), -((2 * TILE) + (165 / 2)))); // centered on second-last row

                // april tags on the backdrop are 3.5" apart / 90mm
                if (target == 1) {
                    //blue far backdrop one
                    actions.add(new ActionAprilLock(rear_cam, 1));
                    actions.add(new ActionMove(((3 * TILE) + (165 / 2)), -((2 * TILE) - 170 - 90 - 90)));
                } else if (target == 2) {
                    // blue far backdrop 2
                    actions.add(new ActionAprilLock(rear_cam, 2));
                    actions.add(new ActionMove(((3 * TILE) + (165 / 2)), -((2 * TILE) - 170 - 90)));
                } else if (target == 3) {
                    //blue far backrop 3
                    actions.add(new ActionMove(((3 * TILE) + (165 / 2)), -((2 * TILE) - 170)));
                    actions.add(new ActionAprilLock(rear_cam, 3));
                    //actions.add(new ActionMove(-((3 * TILE) + (165 / 2) + 193), (925)));
                }
                // "score the pixel" actions (and return arm to start)
                actions.add(new ActionArm("low-scoring"));
                actions.add(new ActionPause(0.1));
                actions.add(new ActionArm("open"));
                //actions.add(new ActionPause(0.2));
               // actions.add(new ActionMove(-((3 * TILE) + (165 / 2) + 150), (925)));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionPause(.2));
                actions.add(new ActionArm("intake"));
            }
        } else if (getZone() == Zone.FAR && getAlliance() == Alliance.RED) {
            if (target == 1) {
                actions.add(new ActionMove(-340, 365));
                //actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(-TILE, (360)));
                actions.add(new ActionMove(-TILE, (165 / 2) + TILE));
            } else if (target == 2) {
                // zone 2
                actions.add(new ActionMove(-165/2, 1.5 * TILE));
                actions.add(new ActionMove(-165/2, 585));
                //actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(-TILE  + (165 / 2), (165 / 2) + TILE));
            } else {
                // zone 3 (under truss)
                actions.add(new ActionMove(-(165/2), 165 / 2 + TILE));
                actions.add(new ActionTurn(-90));
                actions.add(new ActionMove(TILE / 2, 165 / 2 + TILE));
                actions.add(new ActionMove(-150, TILE + 55));
                //actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(-TILE  + (165 / 2), (165 / 2) + TILE));
                actions.add(new ActionTurn(0));
            }

            // no matter what we did with the pixel above, we're in the same position and can go to the board
            // RED

            if (true) {
                actions.add(new ActionMove( -TILE + (165 / 2), (2 * TILE) + 20)); // center lane
                actions.add(new ActionTurn(90));//turn to face the arm towards the backdrop
                actions.add(new ActionMove(((3 * TILE)), (2 * TILE) + 20)); // centered on second-last row

                if (target == 3) {
                    actions.add(new ActionMove(((3 * TILE) + 170), (505)));
                } else if (target == 2) {
                    actions.add(new ActionMove(((3 * TILE) + 170), (655)));
                } else if (target == 1) {
                    actions.add(new ActionMove(((3 * TILE) + 170), (890)));
                }
                // "score the pixel" actions (and return arm to start)
                actions.add(new ActionArm("resting"));
                actions.add(new ActionArm("low-scoring"));
                actions.add(new ActionPause(.2));
                actions.add(new ActionArm("open"));
                actions.add(new ActionPause(.1));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionPause(.2));
                actions.add(new ActionArm("intake"));
            }
        }

        // always at the end we want to do this...
        actions.add(new ActionNothing());
    }

    @Override
    public void loop() {
        double delta = time - last_loop;
        last_loop = time;
        TelemetryPacket pack = new TelemetryPacket();


        if (special_action != null) {
            boolean res = special_action.update(time, drive, intake, arm, telemetry, pack);
            if (res) {
                special_action = null;
            }
        } else {
            if (pipeline.result != ReverseTeamElementPipeline.Result.Unknown) {
                target = 3;
                if (pipeline.result == ReverseTeamElementPipeline.Result.Left) {
                    target = 1;
                } else if (pipeline.result == ReverseTeamElementPipeline.Result.Middle) {
                    target = 2;
                } else if (pipeline.result == ReverseTeamElementPipeline.Result.Right) {
                    target = 3;
                }
                if (actions.size() == 0) {
                    createActions();
                }
                front_cam.stopStreaming();
                telemetry.addData("detected-target", target);
            }
        }

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

        telemetry.addData("x", drive.odo.position_x());
        telemetry.addData("y", drive.odo.position_y());
        telemetry.addData("delta", delta);
        pack.put("x", drive.odo.position_x());
        pack.put("y", drive.odo.position_y());
        pack.put("delta", delta);
        //pack.put("last_x", drive.odo.x_last);
        //pack.put("last_y", drive.odo.y_last);
        //pack.put("par-encoder", drive.odo.par.encoder.getPosition());
        //pack.put("perp-encoder", drive.odo.perp.encoder.getPosition());
        pack.put("target", target);

        // directions (in start position):
        // +x = robot-right
        // -x = robot-left
        // +y = robot forward
        // -y = robot backwards

        // rotations:
        // +90 == turn left (??)
        // -90 == turn right

        // seems to not work if "data" and "drawing" are in the same packet?
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        pack = new TelemetryPacket();

        // actual robot is 407mm square
        double INCHES_TO_MM = 0.03937008;

        // origin to start position (XXX overridable method for this?)
        pack.fieldOverlay().setTranslation(-1 * 24, 3 * 24);

        // do all other drawing in millimeters
        pack.fieldOverlay().setScale(INCHES_TO_MM, INCHES_TO_MM);
        pack.fieldOverlay().setRotation(-Math.toRadians(90));
        // center the drawing in the robot
        //pack.fieldOverlay().setTranslation(-203, 203);
        pack.fieldOverlay()
                .setFill("green")
                .fillCircle(0.0, 0.0, 2.0)
                .setFill("red")
                .fillRect(drive.odo.position_y(), drive.odo.position_x(), 407, 407);

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
