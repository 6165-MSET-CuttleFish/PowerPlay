package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.*
import java.util.concurrent.Callable

class TaskScheduler(val l: LinearOpMode)
{
    //you can just use set state idk why u would use this but it exists for uniformity i guess
    fun scheduleTask(module: Module, state: ModuleState)
    {
        GlobalScope.launch (Dispatchers.Default)
        {
            module.setState(state)
        }
    }
    //schedule a task with a delay(state changes after however many seconds
    fun scheduleTask(module: Module, state: ModuleState, timeMilis: Long)
    {
        GlobalScope.launch (Dispatchers.Default)
        {
            delay(timeMilis)
            module.setState(state)
        }
    }
    //schedule a task to occur only after a certain condition is met
    fun scheduleTask(module: Module, state: ModuleState, startCondition: Callable<Boolean>)
    {
        GlobalScope.launch (Dispatchers.Default)
        {
            while(!startCondition.call()&&!l.isStopRequested)
            {

            }
            module.setState(state)
        }
    }
    //schedule a task to occur however many seconds after a certain condition is met
    fun scheduleTask(module: Module, state: ModuleState, timeMilis: Long, startCondition: Callable<Boolean>)
    {
        GlobalScope.launch (Dispatchers.Default)
        {
            while(!startCondition.call()&&!l.isStopRequested)
            {

            }
            delay(timeMilis)
            module.setState(state)
        }
    }
}