package org.firstinspires.ftc.teamcode.util.moduleUtil

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.*

class TaskScheduler(val l: LinearOpMode)
{

    //schedule a task to occur however many seconds after a certain condition is met
    fun scheduleTask(t: ImprovedTask)
    {
        if(!t.blocking)
        {
            GlobalScope.launch (Dispatchers.Default)
            {
                while(!t.condition.runCondition.call()&&!l.isStopRequested)
                {

                }
                delay(t.delayTime)
                t.action.invoke()
            }
        }
        else
        {
            runBlocking {
                while(!t.condition.runCondition.call()&&!l.isStopRequested)
                {

                }
                delay(t.delayTime)
                t.action.invoke()
            }
        }
    }



    fun scheduleTaskList(t: TaskList)
    {

    }
}