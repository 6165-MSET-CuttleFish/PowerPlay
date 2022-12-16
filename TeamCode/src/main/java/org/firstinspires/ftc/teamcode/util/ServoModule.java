package org.firstinspires.ftc.teamcode.util;

public abstract class ServoModule{

    public ServoWorker w;
    public ModuleState state;

    public ServoModule(){w=new ServoWorker(this);}

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
