package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw
{
    //TODO: Implement a way to check if servo is all the way there to target position

    //temporary values
    static final double OPEN=0;
    static final double CLOSE=0;
    static final double PARTIAL=0;

    ServoEx claw;
    public Claw.State state;
    public enum State
    {
        OPEN, CLOSE, PARTIAL
    }

    public Claw(HardwareMap hardwareMap)
    {
        claw=hardwareMap.get(ServoEx.class, "Claw");
    }

    public void update()
    {
        switch(state)
        {
            case OPEN:
                claw.setPosition(OPEN);
            case CLOSE:
                claw.setPosition(CLOSE);
            case PARTIAL:
                claw.setPosition(PARTIAL);
        }
    }

    public Claw.State getState() {
        return state;
    }

    public void setState(Claw.State state){
        this.state = state;
    }

}
