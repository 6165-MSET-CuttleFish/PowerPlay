package org.firstinspires.ftc.teamcode.moduleUtil

import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.moduleUtil.MotorModule

class MotorWorker(val m: MotorModule)
{
    var delay: Int=0;
    val timer: ElapsedTime = ElapsedTime();

    fun start()
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                checkDelay()
                m.update();
            }
        }
    }

    private fun checkDelay()
    {
        if(delay>0)
        {
            timer.reset()
            while(timer.milliseconds()<delay)
            {
                m.dead()
            }
        }
    }
}