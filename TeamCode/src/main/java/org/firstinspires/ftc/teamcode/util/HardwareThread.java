package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;

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
