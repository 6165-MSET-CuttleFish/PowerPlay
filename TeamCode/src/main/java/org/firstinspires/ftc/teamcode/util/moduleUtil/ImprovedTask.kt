package org.firstinspires.ftc.teamcode.util.moduleUtil

import java.lang.reflect.Constructor
import java.util.concurrent.Callable
import java.util.function.Consumer

class ImprovedTask<T>  @JvmOverloads constructor
    (
    val function: Consumer<T>,
    val param: T,
    @JvmField val delayTime: Long=0,
    @JvmField val startCondition: Callable<Boolean> = Callable{true},
    val continueCondition: Callable<Boolean> = Callable{true},
    val blocking: Boolean=false
    )
