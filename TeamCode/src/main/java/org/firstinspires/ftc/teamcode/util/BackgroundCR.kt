package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.RobotTemp
import org.firstinspires.ftc.teamcode.modules.slides.Slides
import org.firstinspires.ftc.teamcode.modules.turret.Turret

class BackgroundCR(val turret: Turret, val slides: Slides)
{
    fun startHW()
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                turret.update()
            }
        }
        GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                slides.update()
            }
        }
    }
}
