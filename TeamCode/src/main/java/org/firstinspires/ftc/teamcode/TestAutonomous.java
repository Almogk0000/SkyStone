package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TestAutonomous", group = "Tests")
public class TestAutonomous extends LinearOpMode {

    private DcMotor leftMotor_u;
    private DcMotor leftMotor_d;
    private DcMotor rightMotor_u;
    private DcMotor rightMotor_d;
    private Servo armServo;

    private double startArm = 0;
    private double pressA_Arm = 0.5;
    private double gamepad1LeftY = gamepad1.left_stick_y;
    private double gamepad1LeftX = gamepad1.left_stick_x;
    private double gamepad1RightY = gamepad1.right_stick_y;
    private double gamepad1RightX = gamepad1.right_stick_x;


    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor_u = hardwareMap.dcMotor.get("leftMotor_u");
        leftMotor_d = hardwareMap.dcMotor.get("leftMotor_d");
        rightMotor_u = hardwareMap.dcMotor.get("rightMotor_u");
        rightMotor_d = hardwareMap.dcMotor.get("rightMotor_d");
        armServo = hardwareMap.servo.get("armServo");

        armServo.setPosition(startArm);
        leftMotor_u.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_d.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            Drive(leftMotor_d, -gamepad1LeftY);
            Drive(leftMotor_u, gamepad1LeftX);
            Drive(rightMotor_d, -gamepad1RightY);
            Drive(rightMotor_u, gamepad1RightX);
        }
        if(gamepad2.a){
            armServo.setPosition(pressA_Arm);
        }
    }

    public void Drive(DcMotor motor, double power){
        motor.setPower(power);
    }
}
