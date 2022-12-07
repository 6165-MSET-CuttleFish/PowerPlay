package org.firstinspires.ftc.teamcode.Transfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class vfourb
{
    //temporary values
    public static double INTAKE_POSITION = 1;
    public static double DEPOSIT_POSITION = 0.14;
    public static double PRIMED = 0.86;
    public static double STACK_LOW = 0.63;
    public static double ALIGN_POSITION = 0.27;
    public static double STACK_PRIMED = 0.83;
    public static double VERTICAL = 0.5;
    public static double INIT=0.86;
    public static double OFFSET=0;
    Servo Running;
    Servo Supporting;
    public State state;
    public enum State {
        INTAKE_POSITION,DEPOSIT_POSITION, PRIMED, ALIGN_POSITION, STACK_PRIMED, VERTICAL, STACK_LOW, INIT
    }

    public vfourb(HardwareMap hardwareMap)
    {
        Running = hardwareMap.get(Servo.class, "v4bRun");
        Supporting = hardwareMap.get(Servo.class, "v4bSup");
        setState(State.INIT);
    }

    public void update()
    {
        switch(state)
        {
            case INTAKE_POSITION:
                Running.setPosition(INTAKE_POSITION);
                Supporting.setPosition(1-INTAKE_POSITION/*+OFFSET*/);
                break;
            case DEPOSIT_POSITION:
                Running.setPosition(DEPOSIT_POSITION);
                Supporting.setPosition(1-DEPOSIT_POSITION/*+OFFSET*/);
                break;
            case PRIMED:
                Running.setPosition(PRIMED);
                Supporting.setPosition(1-PRIMED/*+OFFSET*/);
                break;
            case ALIGN_POSITION:
                Running.setPosition(ALIGN_POSITION);
                Supporting.setPosition(1-ALIGN_POSITION/*+OFFSET*/);
                break;
            case STACK_PRIMED:
                Running.setPosition(STACK_PRIMED);
                Supporting.setPosition(1-STACK_PRIMED/*+OFFSET*/);
                break;
            case VERTICAL:
                Running.setPosition(VERTICAL);
                Supporting.setPosition(1-VERTICAL/*+OFFSET*/);
                break;
            case STACK_LOW:
                Running.setPosition(STACK_LOW);
                Supporting.setPosition(1-STACK_LOW/*+OFFSET*/);
                break;
            case INIT:
                Running.setPosition(INIT);
                Supporting.setPosition(1-INIT/*+OFFSET*/);
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
        update();
    }

}