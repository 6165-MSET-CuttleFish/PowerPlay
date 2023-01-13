package org.firstinspires.ftc.teamcode.modules.moduleUtil;

import java.util.concurrent.Callable;

public abstract class BasicModule {

    public BasicModuleWorker w;
    public BasicModuleState state;

    public BasicModule(){w=new BasicModuleWorker(this);}

    public void setState(BasicModuleState s)
    {
        state=s;
        w.start();
    }

    public void setState(BasicModuleState s, int delayMilis)
    {
        w.startDelay(delayMilis, s);
    }

    public void setState(BasicModuleState s, Callable<Boolean> startCondition)
    {
        w.startSync(startCondition, s);
    }

    public void setState(BasicModuleState s, Callable<Boolean> startCondition, int delayMilis)
    {
        w.startSync(startCondition, delayMilis, s);
    }

    public BasicModuleState getState()
    {
        return state;
    }

    public abstract void update();
}
