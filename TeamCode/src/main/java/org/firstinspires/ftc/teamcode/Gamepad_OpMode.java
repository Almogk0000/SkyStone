/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of using a controller to move the robot.
 */

@TeleOp(name="Basic: Gamepad OpMode", group="Tests")
public class Gamepad_OpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_b = null;
    private DcMotor rightMotor_f = null;
    private double power = 0.6;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // We assign the hardware map of the motors we declared above.
        leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b  = hardwareMap.get(DcMotor.class, "mBL");
        rightMotor_b = hardwareMap.get(DcMotor.class, "mBR");
        rightMotor_f = hardwareMap.get(DcMotor.class, "mFR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor_b.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_f.setDirection(DcMotor.Direction.REVERSE);
        rightMotor_b.setDirection(DcMotor.Direction.FORWARD);
        rightMotor_f.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(gamepad1.left_stick_y >= 0.1 && gamepad1.left_stick_y >= 1.0){
            StopDrive(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f);
            telemetry.clearAll();
            DriveForward(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f, power);
            Telemetry.Line line1 = telemetry.addLine("Moving forward, all motors are on 0.6");
        }else if(gamepad1.left_stick_y >= -0.1 && gamepad1.left_stick_y >= -1.0){
            StopDrive(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f);
            telemetry.clearAll();
            DriveBackwards(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f, -power);
            Telemetry.Line line2 = telemetry.addLine("Moving back, all motors are on -0.6");
        }else if(gamepad1.left_stick_x >= -0.1 && gamepad1.left_stick_x >= -1.0){
            StopDrive(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f);
            telemetry.clearAll();
            DriveLeft(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f, -power);
            Telemetry.Line line3 = telemetry.addLine("Moving left, all motors are on -0.6");
        }else if(gamepad1.left_stick_x >= 0.1 && gamepad1.left_stick_x >= 1.0){
            StopDrive(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f);
            telemetry.clearAll();
            DriveRight(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f, power);
            Telemetry.Line line4 = telemetry.addLine("Moving right, all motors are on 0.6");
        }else{
            StopDrive(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f);
        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        StopDrive(leftMotor_b, leftMotor_f, rightMotor_b, rightMotor_f);
    }

    public void DriveForward(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, double power){
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }
    public void DriveBackwards(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, double power){
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }
    public void DriveLeft(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, double power){
        motor1.setPower(-power);
        motor2.setPower(power);
        motor3.setPower(-power);
        motor4.setPower(power);
    }
    public void DriveRight(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, double power){
        motor1.setPower(power);
        motor2.setPower(-power);
        motor3.setPower(power);
        motor4.setPower(-power);
    }
    public void StopDrive(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4){
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}
