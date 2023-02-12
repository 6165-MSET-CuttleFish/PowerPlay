package org.firstinspires.ftc.teamcode.util.moduleUtil;

import java.util.concurrent.Callable;

public abstract class HwModule
{
    public abstract void setState(ModuleState s);
    public abstract boolean isBusy();

    public Task task(ModuleState m)
    {
        return new Task(this, m);
    }
    public Task task(ModuleState m, long delay)
    {
        return new Task(this, m, delay);
    }
    public Task task(ModuleState m, RunCondition startCondition)
    {
        return new Task(this, m, startCondition);
    }
    public Task task(ModuleState m, long delay, RunCondition startCondition)
    {
        return new Task(this, m, delay, startCondition);
    }
    public RunCondition waitOnTaskCompletion()
    {
        return new RunCondition(()->this.isBusy());
    }
}
