package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretThread extends Thread
{
    ElapsedTime t;
    Turret turret;
    boolean online=false;

    public TurretThread (Turret turret)
    {
        this.turret=turret;
        online=true;
    }

    public void kill()
    {
        online=false;
    }

    @Override
    public void run()
    {
        t=new ElapsedTime();
        while(online)
        {
            turret.update(t.seconds());
            t.reset();
        }
    }

}
