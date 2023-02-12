package org.firstinspires.ftc.teamcode.util.moduleUtil;

import java.util.concurrent.Callable;

public class RunCondition
{
    Callable<Boolean > runCondition;

    public RunCondition(Callable<Boolean> runCondition)
    {
        this.runCondition=runCondition;
    }
}
