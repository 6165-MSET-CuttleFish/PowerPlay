package org.firstinspires.ftc.teamcode.transfer;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class vfourb
{
    //temporary values
    static final double INTAKE_POSITION_RUNNING=0;
    static final double INTAKE_POSITION_SUPPORTING=0;
    static final double DEPOSIT_POSITION_RUNNING=0;
    static final double DEPOSIT_POSITION_SUPPORTING=0;

    ServoEx Running;
    ServoEx Supporting;
    public vfourb.State state;
    public enum State
    {
        INTAKE_POSITION,DEPOSIT_POSITION
    }

    public vfourb(HardwareMap hardwareMap)
    {
        Running = hardwareMap.get(ServoEx.class, "v4brun");
        Supporting = hardwareMap.get(ServoEx.class, "v4bsup");
        setState(State.INTAKE_POSITION);
    }

    public void update()
    {
        switch(state)
        {
            case INTAKE_POSITION:
                Running.setPosition(INTAKE_POSITION_RUNNING);
                Supporting.setPosition(INTAKE_POSITION_SUPPORTING);
            case DEPOSIT_POSITION:
                Running.setPosition(DEPOSIT_POSITION_RUNNING);
                Supporting.setPosition(DEPOSIT_POSITION_SUPPORTING);
        }
    }

    public vfourb.State getState() {
        return state;
    }

    public void setState(vfourb.State state)
    {
        this.state = state;
        update();
    }

}