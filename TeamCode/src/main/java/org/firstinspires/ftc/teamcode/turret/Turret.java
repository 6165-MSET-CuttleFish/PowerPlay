package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret
{

    //temporary values
   DcMotorEx Motore;
    public State state;
    public enum State
    {
        ALIGNING, RESET
    }

    public Turret(HardwareMap hardwareMap)
    {
        Motore = hardwareMap.get(DcMotorEx.class, "turret");
    }

    public void update()
    {
        switch(state)
        {
            case ALIGNING:
                //tbd
            case RESET:
                //tbd
        }
    }

    public State getState() {
        return state;
    }

    public void setState(Turret.State state)
    {
        this.state = state;
        update();
    }

}
