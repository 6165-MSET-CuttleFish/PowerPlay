package org.firstinspires.ftc.teamcode.moduleUtil;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PIDCoeff;
import org.firstinspires.ftc.teamcode.util.PIDControl;

import java.util.List;

//TODO 1. Test?
public abstract class MotorModule
{
    public MotorWorker w;
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
