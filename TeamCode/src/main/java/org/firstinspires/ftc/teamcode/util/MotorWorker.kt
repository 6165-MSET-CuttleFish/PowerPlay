package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.*

class MotorWorker(val m:MotorModule)
{
    var currentJob: Job? = null

    fun start()
    {
        currentJob= GlobalScope.launch(Dispatchers.Main)
        {
            while(isActive)
            {
                m.updateTarget();
                if(m.isBusy)
                {
                    m.update();
                }
            }
        }
    }

    fun interrupt()
    {
        //interrupt the current coroutine
        currentJob?.cancel();
    }
}