package org.firstinspires.ftc.teamcode.moduleUtil

import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.moduleUtil.BasicModule

class BasicModuleWorker(val m: BasicModule)
{
    var currentJob: Job? = null

    fun start()
    {
        currentJob=GlobalScope.launch(Dispatchers.Main)
        {
            m.update();
        }
    }

    fun startDelay(milis: Long)
    {
        currentJob=GlobalScope.launch(Dispatchers.Main)
        {
            delay(milis);
            m.update();
        }
    }

    fun interrupt()
    {
        //interrupt the current coroutine
        currentJob?.cancel();
    }
}