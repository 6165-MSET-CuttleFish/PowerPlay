package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw
{
    //temporary values
    public static double OPEN=0;
    public static double CLOSE=0.35;
    public static double PARTIAL=0;

    Servo claw;
    public Claw.State state;
    public enum State
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

    public void setState(Claw.State state)
    {
        this.state = state;
        update();
    }

}
