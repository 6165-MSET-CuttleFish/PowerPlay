package org.firstinspires.ftc.teamcode.util.moduleUtil

import java.util.concurrent.Callable

class ImprovedTask(
    val action: ((()->Unit?)),
    val delayTime: Long=0,
    val condition: RunCondition=RunCondition(Callable{true}),
    val blocking: Boolean=false)
{

}