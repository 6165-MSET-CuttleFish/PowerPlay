package org.firstinspires.ftc.teamcode.Transfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Module;

@Config
public class vfourb extends Module
{
    //temporary values
    public static double INTAKE_POSITION = 0.95;
    public static double DEPOSIT_POSITION = 0.15;
    public static double PRIMED = 0.7;
    public static double ALIGN_POSITION = 0.34;
    public static double STACK_PRIMED = 0.741;
    public static double VERTICAL = 0.5;
    Servo Running;
    Servo Supporting;
    public State state;
    public enum State {
        INTAKE_POSITION,DEPOSIT_POSITION, PRIMED, ALIGN_POSITION, STACK_PRIMED, VERTICAL
    }

    public vfourb(HardwareMap hardwareMap)
    {
        Running = hardwareMap.get(Servo.class, "v4bRun");
        Supporting = hardwareMap.get(Servo.class, "v4bSup");
        setState(State.INTAKE_POSITION);
    }

    @Override
    public void update()
    {
        switch(state)
        {
            case INTAKE_POSITION:
                Running.setPosition(INTAKE_POSITION);
                Supporting.setPosition(1-INTAKE_POSITION);
                break;
            case DEPOSIT_POSITION:
                Running.setPosition(DEPOSIT_POSITION);
                Supporting.setPosition(1-DEPOSIT_POSITION);
                break;
            case PRIMED:
                Running.setPosition(PRIMED);
                Supporting.setPosition(1-PRIMED);
                break;
            case ALIGN_POSITION:
                Running.setPosition(ALIGN_POSITION);
                Supporting.setPosition(1-ALIGN_POSITION);
                break;
            case STACK_PRIMED:
                Running.setPosition(STACK_PRIMED);
                Supporting.setPosition(1-STACK_PRIMED);
                break;
            case VERTICAL:
                Running.setPosition(VERTICAL);
                Supporting.setPosition(1-VERTICAL);
                break;
        }
    }
    public double runPos(){
        return Running.getPosition();
    }
    public double supPos(){
        return Supporting.getPosition();
    }
    public State getState() {
        return state;
    }

    public void setState(State state)
    {
        this.state = state;
        //update();
    }

}