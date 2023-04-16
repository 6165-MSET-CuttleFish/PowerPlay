package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

//not actually used
@Config
public class AutoalignController
{
    public static double minChange=10;
    public static double defaultLowPower=0.15;
    public static double defaultHighPower=0.3;
    public static double gain=0.0045;
    ElapsedTime t=new ElapsedTime();

    double pastError;
    double highPower;
    double lowPower;

    public double rateOfChangeError;

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
        t.reset();
    }

    public void clearError()
    {
        pastError=0;
        t.reset();
    }

    public double update(double error)
    {
        this.error=error;

        rateOfChangeError=(pastError-error)/t.milliseconds();

        if(Math.abs(rateOfChangeError)<minChange&&Math.abs(error)>10)
        {
            highPower+=gain;
            lowPower+=gain;
        }

        pastError=error;

        t.reset();

        if(Math.abs(error)>30)
            return Math.signum(error)*-highPower;
        else if(Math.abs(error)>10)
            return Math.signum(error)*-lowPower;
        else
            return 0;
    }
}
