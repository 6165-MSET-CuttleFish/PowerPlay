package org.firstinspires.ftc.teamcode.modules.moduleUtil

import kotlinx.coroutines.*
import java.util.concurrent.Callable

class BasicModuleWorker(val m: BasicModule)
{

    fun start()
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            m.update();
        }
    }

    fun startDelay(milis: Long, state: BasicModuleState)
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            delay(milis);
            m.setState(state);
        }
    }

    fun startSync(startCondition: Callable<Boolean>, state: BasicModuleState)
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(!startCondition.call())
            {

            }
            m.setState(state);
        }
    }

    fun startSync(startCondition: Callable<Boolean>, delay:Long, state: BasicModuleState)
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(!startCondition.call())
            {

            }
            delay(delay);
            m.setState(state);
        }
    }
}