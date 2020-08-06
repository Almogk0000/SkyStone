package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Move;


/**
 * This file contains an example of a gamepad movement test with the joysticks.
 */

@TeleOp(name="Basic: Gamepad OpMode", group="Tests")
public class Gamepad_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
	private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;
	private double power = 0.6;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
		leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
		leftMotor_b = hardwareMap.get(DcMotor.class, "mFR");
		rightMotor_f  = hardwareMap.get(DcMotor.class, "mBL");
		rightMotor_b = hardwareMap.get(DcMotor.class, "mBR");

        // Reversing the left motors.
		leftMotor_f.setDirection(DcMotor.Direction.FORWARD);
		leftMotor_b.setDirection(DcMotor.Direction.FORWARD);
		rightMotor_f.setDirection(DcMotor.Direction.REVERSE);
		rightMotor_b.setDirection(DcMotor.Direction.REVERSE);
		
		//Building the move constructor.
		Move motors = new Move(leftMotor_f, leftMotor_b, rightMotor_f, rightMotor_b, power);
		
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
			
			if(gamepad1.left_bumper){
				motors.TurnLeft(power);
			}else if(gamepad1.right_bumper){
				motors.TurnRight(power);
			}else{
				motors.StopMotors();
			}
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveY = gamepad1.left_stick_y;
			double driveX = gamepad1.left_stick_x;
            double turnX  =  gamepad1.right_stick_x;
			double turnY  =  gamepad1.right_stick_y;

            // Send power to wheels
            if(driveY  > 0 || gamepad1.dpad_up){
				motors.DriveForward(power);
			}else if(-driveY  < 0 || gamepad1.dpad_down){
				motors.DriveBackwards(power);
			}else if(driveX  > 0 || gamepad1.dpad_right){
				motors.DriveRight(power);
			}else if(-driveX  < 0 || gamepad1.dpad_left){
				motors.DriveLeft(power);
			}else{
				motors.StopMotors();
			}
			
			if(turnY  > 0 || gamepad1.y){
				motors.DriveDiagonalDownLeft(power);
			}else if(-turnY  < 0 || gamepad1.a){
				motors.DriveDiagonalDownRight(power);
			}else if(turnX  > 0 || gamepad1.x){
				motors.DriveDiagonalUpLeft(power);
			}else if(-turnX  < 0 || gamepad1.b){
				motors.DriveDiagonalUpRight(power);
			}else{
				motors.StopMotors();
			}
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
