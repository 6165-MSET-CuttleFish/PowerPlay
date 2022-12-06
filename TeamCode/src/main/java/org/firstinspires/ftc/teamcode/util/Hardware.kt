package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.Robot


class Hardware (val r: Robot, val l: LinearOpMode)
{
    fun startHW()
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(l.opModeIsActive())
            {
                
            }
        }
    }

}