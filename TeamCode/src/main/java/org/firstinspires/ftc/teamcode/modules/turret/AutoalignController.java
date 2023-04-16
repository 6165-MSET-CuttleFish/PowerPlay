package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoalignController
{
    public static double minChange=10;
    public static double defaultLowPower=0.15;
    public static double defaultHighPower=0.3;
    public static double gain=0.0045;

    double pastError;
    double highPower;
    double lowPower;

    double error;

    public AutoalignController()
    {
        highPower=defaultHighPower;
        lowPower=defaultLowPower;
    }

    public void resetPowers()
    {
        highPower=defaultHighPower;
        lowPower=defaultLowPower;
    }

    public void clearError()
    {
        pastError=0;
    }

    public double update(double error)
    {
        this.error=error;

        if(Math.abs(pastError-error)<minChange&&Math.abs(error)>10)
        {
            highPower+=gain;
            lowPower+=gain;
        }

        pastError=error;

        if(Math.abs(error)>30)
            return Math.signum(error)*-highPower;
        else if(Math.abs(error)>10)
            return Math.signum(error)*-lowPower;
        else
            return 0;
    }
}
