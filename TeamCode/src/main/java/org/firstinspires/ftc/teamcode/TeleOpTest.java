package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="Test:teleop", group="Opmode")
public class TeleOpTest extends OpMode   {

    public DcMotor motor_fl = null;
    public DcMotor motor_fr = null;
    public DcMotor motor_bl = null;
    public DcMotor motor_br = null;



    @Override
    public void init() {
        //hardwareMap.Set
        motor_fl = (DcMotor) hardwareMap.get("FL_Drive");
        motor_fr = (DcMotor) hardwareMap.get("FR_Drive");
        motor_bl = (DcMotor) hardwareMap.get("BL_Drive");
        motor_br = (DcMotor) hardwareMap.get("BR_Drive");
        motor_fl.setDirection(DcMotor.Direction.REVERSE);
        motor_bl.setDirection(DcMotor.Direction.REVERSE);
        motor_fl.setDirection(DcMotor.Direction.FORWARD);
        motor_fr.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void start() {


    }


    @Override
    public void loop() {
        /*
        telemetry.addData("elapsedTime", "%.3f",time);
        if (gamepad1.a) {
            motor_fl.setPower(0.2);
            //positive power is "backwards" relative to the robot
        } else {
            motor_fl.setPower(0);
        }
        telemetry.update();
         */
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        motor_fl.setPower(leftFrontPower);
        motor_fr.setPower(rightFrontPower);
        motor_bl.setPower(leftBackPower);
        motor_br.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + time.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }

}
