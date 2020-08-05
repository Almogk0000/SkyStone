
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file will test the gamepad to make the robot move with the joysticks.
 */

@TeleOp(name="Basic: 90 Degree OpMode", group="Tests")
public class NinetyDegree_OpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_b = null;
    private DcMotor rightMotor_f = null;
    private double powerX = 0.1;
    private double powerY = 0.6;
    private int rotation = 90;

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
        setMotors((float) powerX, (float) powerY, (float) rotation, rightMotor_f, rightMotor_b, leftMotor_f, leftMotor_b);
        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        StopDrive(rightMotor_b, rightMotor_f, leftMotor_b, leftMotor_f);
    }

    void setMotors(float x, float y, float rot, DcMotor motorFR, DcMotor motorRL, DcMotor motorFL, DcMotor motorRR)  //sets the motor speeds given an x, y and rotation value
    {
        float theta = (float) Math.atan((x/y) - 3.14159/4);  //finds the angle of the joystick and turns it by pi/4 radians or 45 degrees
        rot*=.5;  //scales rotation factor
        float magnitude = (float) Math.sqrt((x*x)+(y*y));  //finds the magnitude of the joystick input by the Pythagorean theorem
        magnitude = magnitude*(100/127) - rot; // subtracts rot from the magnitude to make room for it and scales the magnitude
        float newX = (float) (Math.cos(theta)*magnitude); //finds the input to one set of wheels
        float newY = (float) (Math.sin(theta)*magnitude); //finds the input to the other set of wheels
        //from here on is just setting motor values
        motorFR.setPower(rot + newX);
        motorRL.setPower(rot - newX);
        motorFL.setPower(rot - newY);
        motorRR.setPower(rot + newY);
    }

    public void StopDrive(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4){
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

}
