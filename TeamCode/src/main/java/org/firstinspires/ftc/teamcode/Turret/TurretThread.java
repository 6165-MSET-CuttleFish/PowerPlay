package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretThread extends Thread
{
    LinearOpMode l;
    Turret turret;

    public TurretThread (Turret turret, LinearOpMode l)
    {
        this.turret=turret;
        this.l=l;
    }



    @Override
    public void run()
    {
        while(l.opModeIsActive())
        {
            turret.update();
        }
    }

}
