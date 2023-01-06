package org.firstinspires.ftc.teamcode.util;

public class PIDControl
{
    PIDCoeff coeff;
    double iSumMax;
    double stabThresh;

    double integralSum;
    double prevError;
    double prevTarget;

    double motorOil;

    public PIDControl(PIDCoeff coeff, double iSumMax, double stabThresh)
    {
        this.coeff=coeff;
        this.iSumMax=iSumMax;
        this.stabThresh=stabThresh;

        integralSum=0;
        prevError=0;
    }

    public void gainSchedule(PIDCoeff coeff)
    {
        this.coeff=coeff;
    }

    public double calculate(double current, double target, double time)
    {
        double error=target-current;
        double errorChange=error-prevError;
        double derivative=errorChange/time;
        integralSum+=errorChange*time;

        if(Math.abs(integralSum)>iSumMax)
        {
            double sign=Math.signum(integralSum);
            integralSum=sign*iSumMax;
        }

        if(prevTarget!=target)
        {
            integralSum=0;
        }

        if(derivative>stabThresh)
        {
            motorOil=(coeff.kp*error)+(coeff.kd*derivative);
        }
        else
        {
            motorOil=(coeff.kp*error)+(coeff.kd*derivative)+(coeff.ki*integralSum);
        }

        prevError=error;
        prevTarget=target;

        return motorOil;
    }

}
