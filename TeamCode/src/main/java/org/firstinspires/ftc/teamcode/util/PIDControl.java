package org.firstinspires.ftc.teamcode.util;

public class PIDControl
{
    double kp;
    double ki;
    double kd;
    double iSumMax;
    double stabThresh;

    double integralSum;
    double prevError;
    double prevTarget;

    double motorOil;

    public PIDControl(PIDCoeff coeff)
    {
        kp=coeff.kp;
        ki=coeff.ki;
        kd=coeff.kd;
        iSumMax=coeff.iSumMax;
        stabThresh=coeff.stabThresh;

        integralSum=0;
        prevError=0;
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
            motorOil=(kp*error)+(kd*derivative);
        }
        else
        {
            motorOil=(kp*error)+(kd*derivative)+(ki*integralSum);
        }

        prevError=error;
        prevTarget=target;

        return motorOil;
    }

}
