package org.firstinspires.ftc.teamcode.Modules.moduleUtil

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

    fun startDelay(milis: Long, state: ModuleState)
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            delay(milis);
            m.setState(state);
        }
    }

    fun startSync(startCondition: Callable<Boolean>, state: ModuleState)
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(!startCondition.call())
            {

            }
            m.setState(state);
        }
    }

    fun startSync(startCondition: Callable<Boolean>, delay:Long, state:ModuleState)
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