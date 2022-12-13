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
        w.interrupt();
        state=s;
        w.start();
    }

    public void setState(ModuleState s, int delayMilis)
    {
        w.interrupt();
        state=s;
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
