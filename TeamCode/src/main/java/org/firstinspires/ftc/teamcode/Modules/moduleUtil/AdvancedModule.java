package org.firstinspires.ftc.teamcode.Modules.moduleUtil;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.Callable;

//TODO 1. Test? 2. Functioning PID w/ gain scheduling
public abstract class AdvancedModule
{
    public AdvancedModuleWorker w;
    public ModuleState state;
    public List<DcMotor> motors;
    public double posAtZero;
    public double targetPos;
    public double manualPower;
    public ElapsedTime timer;

    public void setState(ModuleState s)
    {
        timer.reset();
        state=s;
    }

    public void setState(ModuleState s, int delayMilis)
    {
        w.setStateDelay(delayMilis, s);
    }

    public void setState(ModuleState s, Callable<Boolean> startCondition)
    {
        w.setStateSync(startCondition, s);
    }

    public void setState(ModuleState s, Callable<Boolean> startCondition, int delayMilis)
    {
        w.setStateSync(startCondition, delayMilis, s);
    }

    public ModuleState getState() {return state;}

    public void updateTarget()
    {
        if(resetPressed())
            posAtZero=currentPos();
        if(state.getValue()!=null)
            targetPos=state.getValue()+posAtZero;
    }

    public void setManualPower(double manualPower) { this.manualPower=manualPower;}

    public void update()
    {
        updateTarget();
        for(DcMotor m: motors){m.setPower(motorPower());}
        timer.reset();
    }
    public abstract double motorPower();
    public abstract boolean resetPressed();
    public abstract double currentPos();
}
