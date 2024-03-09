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
    OpenCvWebcam megacam;
    int target = -1;
    double last_loop;

    public enum Zone {NEAR, FAR};
    public enum Alliance {RED, BLUE};

    public abstract Zone getZone();
    public abstract Alliance getAlliance();

    public static final double TILE = 610; // 24inches = 610mm

    boolean get_white = false;
    GamepadEx gp;


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
        gp = new GamepadEx(gamepad1);
        actions = new LinkedList<ActionBase>();

        pipeline = new ReverseTeamElementPipeline(getAlliance() == Alliance.RED);
        // front_cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam_1"));

        // front_cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        //     @Override
        //     public void onOpened() {
        //         front_cam.startStreaming(800, 600, OpenCvCameraRotation.SENSOR_NATIVE);
        //     }

        //     @Override
        //     public void onError(int errorCode) {
        //     }
        // });

        megacam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "megacam"));
        megacam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                megacam.startStreaming(800, 600, OpenCvCameraRotation.SENSOR_NATIVE);
                megacam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        arm = new Arm(hardwareMap);
        drive = new Drive(hardwareMap, null, arm, getAlliance() == Alliance.RED);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void init_loop() {
        gp.readButtons();
        if (gp.wasJustPressed(GamepadKeys.Button.A)) {
            get_white = !get_white;
        }

        telemetry.addData("result", pipeline.result);
        telemetry.addData("get white", get_white);
    }

    @Override
    public void start() {
        resetRuntime();
        drive.start();
        drive.imu.resetYaw();
        intake_is_up = true;
        last_loop = time;
        special_action = null;//new ActionInitialIntake();

        // jan 30 2024, changed how we reset after seeing "that one weird arm thing" again; this seems to work
        arm.reset();
    }

    protected void createActions() {
        // We can call getZone() or getAlliance() to figure out which place we started in
        // ...also, "target" is valid when we get here (will be 1, 2 or 3).

        // negative y is robot-forward, negative x is robot-right

        //actions.add(new ActionArm("close"));

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
                actions.add(new ActionMove((-165/2), -(TILE  + 690)));
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
                actions.add(new ActionMove(-160, -(TILE + 20)));
                actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(-250, -(TILE*2 + (165/2))));
            }

            // no matter what we did with the pixel above, we're in the same position and can go to the board

            if (true) {
                actions.add(new ActionTurn(90)); //turn to face the arm towards the backdrop
                actions.add(new ActionMove(((3 * TILE) + (165 / 2)), -((2 * TILE) + (165 / 2)))); // centered on second-last row


                // negative y is robot-forward
                // measured
                // position 1: 20, position 2: 28.5, position 3: 33
                // 508, 724, 838
                double board_position_front = (3 * TILE) + 150;
                double horiz = -495;
                // april tags on the backdrop are 3.5" apart / 90mm
                if (target == 1) {
                    //blue far backdrop one
                    ///actions.add(new ActionAprilLock(rear_cam, 1));
                    actions.add(new ActionMove(board_position_front, -510));
                    horiz = -495;
                } else if (target == 2) {
                    // blue far backdrop 2
                    ///actions.add(new ActionAprilLock(rear_cam, 2));
                    actions.add(new ActionMove(board_position_front, -724));
                    horiz = -724;
                } else if (target == 3) {
                    //blue far backrop 3
                    horiz = -855;
                    actions.add(new ActionMove(board_position_front, -855));
                    ///actions.add(new ActionAprilLock(rear_cam, 3));
                    //actions.add(new ActionMove(-((3 * TILE) + (165 / 2) + 193), (925)));
                }
                // "score the pixel" actions (and return arm to start)
                actions.add(new ActionArm("low-scoring"));
                actions.add(new ActionPause(0.1));
                actions.add(new ActionArm("open"));
                actions.add(new ActionPause(0.2));
                actions.add(new ActionMove(board_position_front - 50, horiz));
                actions.add(new ActionMove(board_position_front - 50, -900));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionPause(.2));
                actions.add(new ActionArm("intake"));
            }
        } else if (getZone() == Zone.FAR && getAlliance() == Alliance.RED) {
            if (target == 1) {
                // blue-side initial bits of motion .. spit out purple pixel
                // zone 1
                actions.add(new ActionMove(385, -(TILE + TILE)));
                actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                // "common" tile? 3 out -- just past the "scoring" tile
                actions.add(new ActionMove(150, -(TILE * 2 + (165 / 2))));
            } else if (target == 2) {
                // blue-side initial bits of motion .. spit out purple pixel
                // zone 2
                actions.add(new ActionMove((165 / 2), -(TILE + 700)));
                actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(150, -(TILE * 2 + (165 / 2))));
            } else {
                // blue-side initial bits of motion .. spit out purple pixel
                // zone 3
                actions.add(new ActionMove(165 / 2, -(TILE + 15)));
                actions.add(new ActionTurn(90));
                actions.add(new ActionMove(160, -(TILE + 30)));
                actions.add(new ActionIntake(false));
                actions.add(new ActionSuck(false));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionIntake(true, true));
                actions.add(new ActionMove(250, -(TILE * 2 + (165 / 2))));
            }

            // no matter what we did with the pixel above, we're in the same position and can go to the board

            if (true) {
                actions.add(new ActionTurn(-90)); //turn to face the arm towards the backdrop
                actions.add(new ActionMove(-((3 * TILE) + (165 / 2)), -((2 * TILE) + (165 / 2)))); // centered on second-last row


                // negative y is robot-forward
                // measured
                // position 1: 34", position 2:28.75", position 3:19.5"
                //866, 730, 495
                double board_position_horiz = -866; // furthest from wall
                double board_position_front = -((3 * TILE) + 150);
                // april tags on the backdrop are 3.5" apart / 90mm
                if (target == 3) {
                    //blue far backdrop one
                    ///actions.add(new ActionAprilLock(rear_cam, 1));
                    actions.add(new ActionMove(board_position_front, -495));
                } else if (target == 2) {
                    // blue far backdrop 2
                    ///actions.add(new ActionAprilLock(rear_cam, 2));
                    actions.add(new ActionMove(board_position_front, -730));
                } else if (target == 1) {
                    //blue far backrop 3
                    actions.add(new ActionMove(board_position_front, -866));
                    ///actions.add(new ActionAprilLock(rear_cam, 3));
                    //actions.add(new ActionMove(-((3 * TILE) + (165 / 2) + 193), (925)));
                }
                // "score the pixel" actions (and return arm to start)
                actions.add(new ActionArm("low-scoring"));
                actions.add(new ActionPause(0.1));
                actions.add(new ActionArm("open"));
                actions.add(new ActionPause(0.2));
                actions.add(new ActionMove(board_position_front + 50, board_position_horiz));
                actions.add(new ActionPause(0.5));
                actions.add(new ActionMove(board_position_front + 50, board_position_horiz));
                actions.add(new ActionArm("resting"));
                actions.add(new ActionPause(.2));
                actions.add(new ActionArm("intake"));
            }
        } else if (getZone() == Zone.NEAR && getAlliance() == Alliance.RED) {
            nearSideActions(actions, true);
        } else if (getZone() == Zone.NEAR && getAlliance() == Alliance.BLUE) {
            nearSideActions(actions, false);
        }

        // always at the end we want to do this...
        actions.add(new ActionArm("intake"));
        actions.add(new ActionNothing());
    }

    protected void farActions(LinkedList<ActionBase> actions, boolean is_red) {
        // negative y is robot-forward
        // negative x is robot-left (towards board on blue side, away from board on red)
        // "165/2" is "half the leftover space: to center our robot on the tile"

        // we have a "common point" to get to before the april-locker takes over
        ActionMove far_point = new ActionMove(150, -(TILE * 2 + (165 / 2)));
        ActionMove common_point = new ActionMove(150, -(TILE * 2 + (165 / 2)));

        if (target == 1) {
            // this one is "under the truss"
            actions.add(new ActionMove(385, -(TILE*2)));
        } else if (target == 2) {
            actions.add(new ActionMove((165/2), -(TILE  + 700)));
        } else {
            actions.add(new ActionMove(165/2, -(TILE + 15)));
            actions.add(new ActionTurn(90));
            actions.add(new ActionMove(160, -(TILE + 30)));
        }

        // the above moves got us to "spit out the purple pixel"
        // location; then we do that and move to our common point
        spitOutPurple(actions);
        actions.add(new ActionTurn(-90));  // face the board
        actions.add(common_point);

        // lock onto the correct april target
        actions.add(new ActionAprilLock(megacam, target, getAlliance() == Alliance.RED));

        addYellowScoring(actions, 1.0);

        if (get_white) {
            getWhitePixelRedClose(actions, is_red);
            addYellowScoring(actions, 2.5);
        }
    }

    protected void nearSideActions(LinkedList<ActionBase> actions, boolean is_red) {
        // negative y is robot-forward
        // negative x is robot-left (towards board on blue side, away from board on red)
        // "165/2" is "half the leftover space: to center our robot on the tile"

        actions.add(new ActionArm("purple"));

        double mult = -1.0;
        if (!is_red) {
            mult = 1.0;
        }

        // we have a "common point" to get to before the april-locker takes over
        ActionMove common_point = new ActionMove(-TILE, -(TILE + 20));

        if (true) {
            if (target == 1) {
                // this one is "under the truss"
                actions.add(new ActionMove(mult * (165 / 2), -TILE));
                //actions.add(new ActionTurn(-90));
                actions.add(new ActionMove(mult * 175, -(TILE + 20)));
            } else if (target == 2) {
                //actions.add(new ActionMove(mult * (165 / 2), -(TILE + 710)));
                actions.add(new ActionMove(mult * (165 / 2), -560));
            } else {
                //actions.add(new ActionMove(mult * 385, -(TILE + TILE)));
                actions.add(new ActionMove(mult * 330, -330));
            }

            // the above moves got us to "spit out the purple pixel"
            // location; then we do that and move to our common point
            pizzaDeliverPurple(actions);
            //spitOutPurple(actions);
        } else {
            // if we're not doing purple right now, just go ahead a bit so we can turn
            actions.add(new ActionMove(0, -200));
        }
        actions.add(new ActionTurn(-90));  // face the board
        actions.add(common_point);

        // lock onto the correct april target
        actions.add(new ActionAprilLock(megacam, target, getAlliance() == Alliance.RED));

        addYellowScoring(actions, 1.0);

        if (get_white) {
            getWhitePixelRedClose(actions, is_red);
            addYellowScoring(actions, 1.5);
        }
    }

    protected void getWhitePixelRedClose(LinkedList<ActionBase> actions, boolean is_red) {
        // this works for "red, close-side"
        // might want to make this "the common spot" too?
        double mult = -1.0;
        if (!is_red) {
            mult = 1.0;
        }

        actions.add(new ActionMove(mult * TILE, -120));
        actions.add(new ActionTurn(-90));
        actions.add(new ActionMove((-mult) * (TILE * 2), -120));
        actions.add(new ActionMove((-mult) * (TILE * 2), -(693))); // 26.5" == 673mm == wall to first stack
        // get the pixel "somehow"
        actions.add(new ActionSuck(true, 4.0));
        //actions.add(new ActionPause(2.0));
        // just pause for now
        actions.add(new ActionMove((-mult) * (TILE * 2), -120));
        actions.add(new ActionMove(mult * TILE, -120));
        // back to "common point"
        actions.add(new ActionMove(mult * TILE, -(TILE + 20)));
        //actions.add(new ActionAprilLock(megacam, 2, getAlliance() == Alliance.RED));
    }

    protected void spitOutPurple(LinkedList<ActionBase> actions) {
        actions.add(new ActionIntake(false));
        actions.add(new ActionSuck(false));
        actions.add(new ActionArm("resting"));
        actions.add(new ActionIntake(true, true));
    }

    protected void pizzaDeliverPurple(LinkedList<ActionBase> actions) {
        actions.add(new ActionArm("purple"));
        actions.add(new ActionArm("spit-out", 1.2));
        actions.add(new ActionArm("spit-stop", 0.1));
        actions.add(new ActionArm("high-scoring", 1.0));
    }

    protected void addYellowScoring(LinkedList<ActionBase> actions, double how_long) {
        if (true) {
            // once we're locked, we score the pixel
            actions.add(new ActionArm("high-scoring"));
            actions.add(new ActionArm("low-scoring"));
            actions.add(new ActionPause(0.1));
            actions.add(new ActionArm("spit-out", how_long));
            actions.add(new ActionArm("spit-stop", 0.1));
            // we actually probably want an "ActionMoveBack(200mm)" or
            // similar; that is, move "positive x" but whatever our Y
            // is currntly at

            actions.add(new ActionArm("resting"));
            actions.add(new ActionPause(.05));
            actions.add(new ActionArm("intake"));
        }

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
                //fixme front_cam.stopStreaming();
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
