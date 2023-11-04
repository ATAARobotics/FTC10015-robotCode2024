package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="Autonomous LEFT", group="Autonomous")
public class AutonomousLeft extends OpMode {

    private Drive drive = null;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
    }

    @Override
    public void start(){
        drive.start();
        drive.imu.resetYaw();
    }

    @Override
    public void loop() {
        if (time < 1.0) {
            drive.robotInputs(0.0, -0.5);
        }else if (time < 4.0) {
            drive.robotInputs(-0.5, 0);
        } else {
            drive.robotInputs(0, 0);
        }
        drive.loop(time);
    }
}
