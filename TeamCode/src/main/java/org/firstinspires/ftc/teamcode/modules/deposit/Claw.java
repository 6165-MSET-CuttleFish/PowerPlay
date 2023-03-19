package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;

@Config
public class Claw extends HwModule
{
    //temporary values
    public static double OPEN_WIDE = 0.3;
    public static double OPEN = 0.4;
    public static double CLOSE = 0.62;
    public static double PARTIAL=0;

    Servo claw, pole;
    public State state = State.OPEN;
    public Pole poleState = Pole.UP;
    @Override
    public void setState(ModuleState s)
    {
        if(s.getClass()==Claw.State.class)
        {
            state=(State)s;
        }
        update();
    }
    public void setPoleState(ModuleState s)
    {
        if(s.getClass()==Claw.Pole.class)
        {
            poleState=(Pole)s;
        }
        update();
    }
    @Override
    public boolean isBusy() {
        return false;
    }
    public static double UP = 0, DOWN = 1;
    public enum Pole implements ModuleState
    {
        UP,DOWN
    }
    public enum State implements ModuleState
    {
        OPEN, CLOSE, PARTIAL, OPEN_WIDE
    }

    public Claw()
    {
        claw = Context.hardwareMap.get(Servo.class, "claw");
        pole = Context.hardwareMap.get(Servo.class, "pole");
        setState(State.OPEN);
        setPoleState(Pole.UP);
    }

    public void update()
    {
        switch(poleState)
        {
            case UP:
                pole.setPosition(UP);
                break;
            case DOWN:
                pole.setPosition(DOWN);
                break;
        }
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
            case OPEN_WIDE:
                claw.setPosition(OPEN_WIDE);
                break;
        }
    }

    public Claw.State getState() {
        return state;
    }

}
