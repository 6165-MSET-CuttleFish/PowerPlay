package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Turret.Turret;

import java.util.List;

public class HardwareThread extends Thread
{
    LinearOpMode l;
    Robot r;
    List<Module> modules;

    public HardwareThread (LinearOpMode l, Robot r)
    {
        this.l=l;
        this.r=r;
        modules=r.getModules();
    }

    @Override
    public void run()
    {
        while(l.opModeIsActive())
        {
            l.telemetry.addData("working", "plz");
            l.telemetry.update();
            for(Module m:modules)
            {
                m.update();
            }
            //r.updateManual();
            //r.update();
        }
    }
}
