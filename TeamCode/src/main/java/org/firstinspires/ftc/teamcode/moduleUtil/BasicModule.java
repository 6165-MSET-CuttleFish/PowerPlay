package org.firstinspires.ftc.teamcode.moduleUtil;

public abstract class BasicModule {

    public BasicModuleWorker w;
    public ModuleState state;

    public BasicModule(){w=new BasicModuleWorker(this);}

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

    public abstract void update();
}
