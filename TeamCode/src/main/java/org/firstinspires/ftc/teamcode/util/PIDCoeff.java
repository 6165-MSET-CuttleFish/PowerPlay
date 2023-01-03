package org.firstinspires.ftc.teamcode.util;

public class PIDCoeff
{
    double kp;
    double ki;
    double kd;
    double iSumMax;
    double stabThresh;


    //implement low pass later
    public PIDCoeff(double kp, double ki, double kd, double iSumMax, double stabThresh)
    {
        this.kp=kp;
        this.ki=ki;
        this.kd=kd;
        this.iSumMax=iSumMax;
        this.stabThresh=stabThresh;
    }
}
