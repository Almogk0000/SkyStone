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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This file contains the code to make every motor run for 2 seocnds, stop and pass on to the next motor for testing.
 */

@TeleOp(name="Basic: Test001 OpMode", group="Tests")
public class Test001_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_b = null;
    private DcMotor rightMotor_f = null;
    private double power = .45;
    private long runFor = 2000;
    private long delay = 1000;

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

        while (true){
            Drive(leftMotor_f, power, runFor);
            Telemetry.Line line1 = telemetry.addLine("Motor Running: left-motor-front");
            telemetry.update();
            WaitXSeconds(delay);
            telemetry.removeLine(line1);
            Drive(leftMotor_b, power, runFor);
            Telemetry.Line line2 = telemetry.addLine("Motor Running: left-motor-back");
            telemetry.update();
            WaitXSeconds(delay);
            telemetry.removeLine(line2);
            Drive(rightMotor_f, power, runFor);
            Telemetry.Line line3 = telemetry.addLine("Motor Running: right-motor-front");
            telemetry.update();
            WaitXSeconds(delay);
            telemetry.removeLine(line3);
            Drive(rightMotor_b, power, runFor);
            Telemetry.Line line4 = telemetry.addLine("Motor Running: right-motor-back");
            telemetry.update();
            WaitXSeconds(delay);
            telemetry.removeLine(line4);
        }
    }

    public void Drive(DcMotor motor1, double power, long time) throws InterruptedException{
        motor1.setPower(power);
        Thread.sleep(time);
    }
    public void WaitXSeconds(long time) throws InterruptedException {
        Thread.sleep(time);
    }
}
