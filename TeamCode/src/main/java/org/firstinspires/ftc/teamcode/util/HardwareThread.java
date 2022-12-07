package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Turret.Turret;

public class HardwareThread extends Thread
{
    LinearOpMode l;
    Turret turret;
    Slides slides;

    public HardwareThread (Turret turret, Slides slides, LinearOpMode l)
    {
        this.turret=turret;
        this.slides=slides;
        this.l=l;
    }

    @Override
    public void run()
    {
        while(!l.isStopRequested())
        {
            turret.update();
            //slides.checkLimit();
        }
    }
}
