/* 
 * This file will use the gamepad1 buttons to move.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file will make the robot move depending on the button pressed.
 */

@TeleOp(name="Basic: Movement OpMode", group="Linear Opmode")
public class Basic_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_b = null;
    private DcMotor rightMotor_f = null;
    private double power = .45;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // We assign the hardware map of the motors we declared above.
        leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b  = hardwareMap.get(DcMotor.class, "mBL");
        rightMotor_b = hardwareMap.get(DcMotor.class, "mBR");
        rightMotor_f = hardwareMap.get(DcMotor.class, "mFR");

        // Setting directions for the motors.
        rightMotor_f.setDirection(DcMotor.Direction.FORWARD);
        rightMotor_b.setDirection(DcMotor.Direction.FORWARD);
        leftMotor_f.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_b.setDirection(DcMotor.Direction.REVERSE);

        Move motors = new Move(leftMotor_f, leftMotor_b, rightMotor_f, rightMotor_b, power);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a == true){
                motors.DriveBackwards(power);
            }else if(gamepad1.y == true){
                motors.DriveForward(power);
            }else if(gamepad1.b == true){
                motors.DriveRight(power);
            }else if(gamepad1.x == true){
                motors.DriveLeft(power);
            }else if(gamepad1.right_stick_button == true){
                motors.TurnRight(power);
            }else if(gamepad1.left_stick_button == true){
                motors.TurnLeft(power);
            }else if(gamepad1.dpad_up == true){
                motors.DriveDiagonalUpLeft(power);
            }else if(gamepad1.dpad_down == true){
                motors.DriveDiagonalUpRight(power);
            }else if(gamepad1.dpad_left == true){
                motors.DriveDiagonalDownLeft(power);
            }else if(gamepad1.dpad_right == true){
                motors.DriveDiagonalDownRight(power);
            }else{
                motors.StopMotors();
            }
        }
    }
}
