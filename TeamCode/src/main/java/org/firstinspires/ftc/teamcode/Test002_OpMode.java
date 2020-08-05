package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * This file contains the code to make the robot go in a left up diagonal way for 3 seconds.
 */

@TeleOp(name="Basic: Test002 OpMode", group="Tests")
public class Test002_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_b = null;
    private DcMotor rightMotor_f = null;
    private double power = .3;
    private long runFor = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Drive(leftMotor_f, leftMotor_b, rightMotor_b, rightMotor_f, power, runFor);
        StopDrive(leftMotor_f, leftMotor_b, rightMotor_b, rightMotor_f);
    }

    public void Drive(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, double power, long time) throws InterruptedException{
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
        Thread.sleep(time);
    }
    public void StopDrive(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4){
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}
