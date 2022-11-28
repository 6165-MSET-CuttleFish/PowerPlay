package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Turret.Turret;

public class HardwareThread extends Thread
{
    LinearOpMode l;
    Robot r;

    public HardwareThread (LinearOpMode l, Robot r)
    {
        this.l=l;
        this.r=r;
    }

    @Override
    public void run()
    {
        while(l.opModeIsActive())
        {
            for(Module m:r.modules)
            {
                m.update();
            }
            r.updateManual();
        }
    }
}
