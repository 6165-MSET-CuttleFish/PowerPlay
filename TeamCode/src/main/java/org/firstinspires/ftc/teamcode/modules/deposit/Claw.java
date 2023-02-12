package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;

@Config
public class Claw extends HwModule
{
    //temporary values
    public static double OPEN = 0.82;
    public static double CLOSE = 0.95;
    public static double PARTIAL=0;

    Servo claw;
    public Claw.State state;

    @Override
    public void setState(ModuleState s)
    {
        if(s.getClass()==Claw.State.class)
        {
            state=(State)s;
        }
        update();
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    public enum State implements ModuleState
    {
        OPEN, CLOSE, PARTIAL
    }

    public Claw(HardwareMap hardwareMap)
    {
        claw = hardwareMap.get(Servo.class, "claw");
        setState(State.OPEN);
    }

    public void update()
    {
        switch(state)
        {
            case OPEN:
                claw.setPosition(OPEN);
                break;
            case CLOSE:
                claw.setPosition(CLOSE);
                break;
            case PARTIAL:
                claw.setPosition(PARTIAL);
                break;
        }
    }

    public Claw.State getState() {
        return state;
    }

}
