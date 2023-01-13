package org.firstinspires.ftc.teamcode.modules.moduleUtil;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.Callable;

//TODO 1. Test? 2. Functioning auto align
public abstract class AdvancedModule
{
    public AdvancedModuleWorker w;
    public AdvancedModuleState state;
    public List<DcMotor> motors;
    public double posAtZero;
    public double targetPos;
    public double manualPower;
    public ElapsedTime timer;

    public void setState(AdvancedModuleState s)
    {
        timer.reset();
        state=s;
    }

    public void setState(AdvancedModuleState s, int delayMilis)
    {
        w.setStateDelay(delayMilis, s);
    }

    public void setState(AdvancedModuleState s, Callable<Boolean> startCondition)
    {
        w.setStateSync(startCondition, s);
    }

    public void setState(AdvancedModuleState s, Callable<Boolean> startCondition, int delayMilis)
    {
        w.setStateSync(startCondition, delayMilis, s);
    }

    public AdvancedModuleState getState() {return state;}

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
