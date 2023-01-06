package org.firstinspires.ftc.teamcode.util;

public class PIDCoeff
{
    double kp;
    double ki;
    double kd;

    //implement low pass later
    public PIDCoeff(double kp, double ki, double kd)
    {
        this.kp=kp;
        this.ki=ki;
        this.kd=kd;
    }
}
