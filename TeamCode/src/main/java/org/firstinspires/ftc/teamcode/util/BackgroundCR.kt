package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Robot

class BackgroundCR(val r: Robot, val l: LinearOpMode)
{
    fun startHW()
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(!l.isStopRequested)
            {
                r.turret.update();
                r.slides.checkLimit();
            }
        }
    }
}