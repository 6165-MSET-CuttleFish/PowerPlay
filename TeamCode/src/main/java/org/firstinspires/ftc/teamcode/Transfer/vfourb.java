package org.firstinspires.ftc.teamcode.Transfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class vfourb
{
    //temporary values
    public static double INTAKE_POSITION=.95;
    public static double DEPOSIT_POSITION=.20;
    public static double PRIMED = .80;
    public static double HIGH_POSITION = 0.38;

    Servo Running;
    Servo Supporting;
    public State state;
    public enum State
    {
        INTAKE_POSITION,DEPOSIT_POSITION, PRIMED, HIGH_POSITION
    }

    public vfourb(HardwareMap hardwareMap)
    {
        Running = hardwareMap.get(Servo.class, "v4bRun");
        Supporting = hardwareMap.get(Servo.class, "v4bSup");
        setState(State.INTAKE_POSITION);
    }

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
            case HIGH_POSITION:
                Running.setPosition(HIGH_POSITION);
                Supporting.setPosition(1-HIGH_POSITION);
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