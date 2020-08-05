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
