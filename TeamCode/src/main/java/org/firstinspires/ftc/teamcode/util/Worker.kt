package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.*

class Worker(val m: HardwareModule)
{
    var currentJob: Job? = null

    fun start()
    {
        currentJob=GlobalScope.launch(Dispatchers.Main)
        {
            m.update();
            while(m.isBusy&&isActive)
            {
                m.update();
            }
        }
    }

    fun startDelay(milis: Long)
    {
        currentJob=GlobalScope.launch(Dispatchers.Main)
        {
            delay(milis);
            m.update();
            while(m.isBusy&&isActive)
            {
                m.update();
            }
        }
    }

    fun interrupt()
    {
        //interrupt the current coroutine
        currentJob?.cancel();
    }
}