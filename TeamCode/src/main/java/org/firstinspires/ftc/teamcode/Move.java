package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Move {
    private DcMotor mFL;
    private DcMotor mBL;
    private DcMotor mFR;
    private DcMotor mBR;
    private double power;

    public Move(DcMotor mFL, DcMotor mBL, DcMotor mFR, DcMotor mBR, double power) {
        this.mFL = mFL;
        this.mBL = mBL;
        this.mFR = mFR;
        this.mBR = mBR;
    }

    public DcMotor getmFL() {
        return mFL;
    }

    public void setmFL(DcMotor mFL) {
        this.mFL = mFL;
    }

    public DcMotor getmBL() {
        return mBL;
    }

    public void setmBL(DcMotor mBL) {
        this.mBL = mBL;
    }

    public DcMotor getmFR() {
        return mFR;
    }

    public void setmFR(DcMotor mFR) {
        this.mFR = mFR;
    }

    public DcMotor getmBR() {
        return mBR;
    }

    public void setmBR(DcMotor mBR) {
        this.mBR = mBR;
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void DriveForward(double power){
        mFL.setPower(power);
        mBL.setPower(power);
        mFR.setPower(power);
        mBR.setPower(power);
    }

    public void DriveBackwards(double power){
        mFL.setPower(-power);
        mBL.setPower(-power);
        mFR.setPower(-power);
        mBR.setPower(-power);
    }

    public void DriveLeft(double power){
        mFL.setPower(-power);
        mBL.setPower(power);
        mFR.setPower(-power);
        mBR.setPower(power);
    }

    public void DriveRight(double power){
        mFL.setPower(power);
        mBL.setPower(-power);
        mFR.setPower(power);
        mBR.setPower(-power);
    }

    public void TurnRight(double power){
        mFL.setPower(power);
        mBL.setPower(power);
        mFR.setPower(-power);
        mBR.setPower(-power);
    }

    public void TurnLeft(double power){
        mFL.setPower(-power);
        mBL.setPower(-power);
        mFR.setPower(power);
        mBR.setPower(power);
    }

    public void DriveDiagonalUpLeft(double power){
        mBL.setPower(power);
        mFR.setPower(power);
    }

    public void DriveDiagonalUpRight(double power){
        mFL.setPower(power);
        mBR.setPower(power);
    }

    public void DriveDiagonalDownLeft(double power){
        mFL.setPower(-power);
        mBR.setPower(-power);
    }
    public void DriveDiagonalDownRight(double power){
        mBL.setPower(-power);
        mFR.setPower(-power);
    }

    public void StopMotors(){
        mFL.setPower(0);
        mBL.setPower(0);
        mFR.setPower(0);
        mBR.setPower(0);
   }
}
