package org.firstinspires.ftc.teamcode.transfer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class vfourb
{
    //temporary values
    static final double INTAKE_POSITION=1;
    static final double DEPOSIT_POSITION=.25;
    static final double PRIMED = .90;

    Servo Running;
    Servo Supporting;
    public State state;
    public enum State
    {
        INTAKE_POSITION,DEPOSIT_POSITION, PRIMED
    }

    public vfourb(HardwareMap hardwareMap)
    {
        Running = hardwareMap.get(Servo.class, "v4brun");
        Supporting = hardwareMap.get(Servo.class, "v4bsup");
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