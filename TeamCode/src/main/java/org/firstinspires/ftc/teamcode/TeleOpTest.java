package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpTest", group = "Tests")
public class TeleOpTest extends LinearOpMode {

    private DcMotor leftMotor_u;
    private DcMotor leftMotor_d;
    private DcMotor rightMotor_u;
    private DcMotor rightMotor_d;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor_u = hardwareMap.dcMotor.get("leftMotor_u");
        leftMotor_d = hardwareMap.dcMotor.get("leftMotor_d");
        rightMotor_u = hardwareMap.dcMotor.get("rightMotor_u");
        rightMotor_d = hardwareMap.dcMotor.get("rightMotor_d");

        leftMotor_u.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_d.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();

        while(opModeIsActive()){
            leftMotor_d.setPower(-gamepad1.left_stick_y);
            leftMotor_u.setPower(gamepad1.left_stick_x);
            rightMotor_d.setPower(-gamepad1.right_stick_y);
            rightMotor_u.setPower(gamepad1.right_stick_x);

            idle();
        }
    }
}
