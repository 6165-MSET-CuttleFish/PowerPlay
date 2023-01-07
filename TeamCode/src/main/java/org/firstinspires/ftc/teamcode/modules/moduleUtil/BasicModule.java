package org.firstinspires.ftc.teamcode.modules.moduleUtil;

import java.util.concurrent.Callable;

public abstract class BasicModule {

    public BasicModuleWorker w;
    public ModuleState state;

    public BasicModule(){w=new BasicModuleWorker(this);}

    public void setState(ModuleState s)
    {
        state=s;
        w.start();
    }

    public void setState(ModuleState s, int delayMilis)
    {
        w.startDelay(delayMilis, s);
    }

    public void setState(ModuleState s, Callable<Boolean> startCondition)
    {
        w.startSync(startCondition, s);
    }

    public void setState(ModuleState s, Callable<Boolean> startCondition, int delayMilis)
    {
        w.startSync(startCondition, delayMilis, s);
    }

    public ModuleState getState()
    {
        return state;
    }

    public abstract void update();
}
