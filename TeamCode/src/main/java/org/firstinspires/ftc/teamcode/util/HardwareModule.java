package org.firstinspires.ftc.teamcode.util;

public abstract class HardwareModule
{
    public Worker w;
    public ModuleState state;

    public HardwareModule()
    {
        w=new Worker(this);
    }

    public void setState(ModuleState s)
    {
        state=s;
        w.interrupt();
        w.start();
    }

    public void setState(ModuleState s, int delayMilis)
    {
        state=s;
        w.interrupt();
        w.startDelay(delayMilis);
    }

    public ModuleState getState()
    {
        return state;
    }

    public boolean isBusy()
    {
        return false;
    }
    public abstract void update();
}
