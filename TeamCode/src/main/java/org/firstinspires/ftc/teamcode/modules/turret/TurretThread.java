package org.firstinspires.ftc.teamcode.modules.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
