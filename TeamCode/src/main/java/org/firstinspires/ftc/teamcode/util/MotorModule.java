package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public abstract class MotorModule
{
    public MotorWorker w;
    public ModuleState state;
    public List<DcMotor> motors;

    public MotorModule()
    {
        motors=new ArrayList<>();
        w=new MotorWorker(this);
        w.start();
    }

    public void setState(ModuleState s)
    {
        state=s;
        for(DcMotor m:motors){m.setMode(state.runMode());}
    }

    public void setState(ModuleState s, int delayMilis)
    {
        state=s;
        for(DcMotor m:motors){m.setMode(state.runMode());}
    }

    public ModuleState getState()
    {
        return state;
    }

    public abstract boolean isBusy();
    public abstract void update();
    public abstract void updateTarget();
}
