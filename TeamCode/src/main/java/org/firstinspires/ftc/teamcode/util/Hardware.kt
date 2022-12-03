package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Robot


class Hardware (val r: Robot, val l: LinearOpMode)
{
    suspend fun startHW() = coroutineScope {
        launch {
            while(!l.isStopRequested)
            {
                r.turret.update();
            }
        }
    }
}