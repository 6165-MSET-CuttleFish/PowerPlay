package org.firstinspires.ftc.teamcode.modules.moduleUtil

import kotlinx.coroutines.*
import java.util.concurrent.Callable

class AdvancedModuleWorker(val m: AdvancedModule)
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

    fun setStateSync(startCondition: Callable<Boolean>, state: ModuleState)
    {
        GlobalScope.launch(Dispatchers.Main)
        {
            while(!startCondition.call())
            {

            }
            m.setState(state);
        }
    }

    fun setStateSync(startCondition: Callable<Boolean>, delay:Long, state:ModuleState)
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