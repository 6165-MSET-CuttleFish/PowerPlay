package org.firstinspires.ftc.teamcode.moduleUtil

import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.moduleUtil.MotorModule

class MotorWorker(val m: MotorModule)
{
    fun start()
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                m.update();
            }
        }
    }

    fun setStateDelay(delay: Long, state: ModuleState)
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            delay(delay)
            m.setState(state)
        }
    }
}