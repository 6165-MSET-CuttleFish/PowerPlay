package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.*

class ServoWorker(val m: ServoModule)
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