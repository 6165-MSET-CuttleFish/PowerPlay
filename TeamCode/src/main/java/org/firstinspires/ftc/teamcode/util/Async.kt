package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch

class Async
{
    companion object {
        fun start(function: () -> (Unit)) {
            GlobalScope.launch(Dispatchers.Main)
            {
                function();
            }
        }
    }
}