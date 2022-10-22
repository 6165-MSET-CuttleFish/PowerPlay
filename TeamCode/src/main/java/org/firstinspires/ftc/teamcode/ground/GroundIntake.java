package org.firstinspires.ftc.teamcode.ground;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GroundIntake
{
    //temporary values
    static final double INTAKING = 1;
    static final double REVERSE = -1;
    static final double OFF = 0;

    CRServo intakeRunning;
    CRServo intakeSupporting;
    public State state;
    public enum State
    {
        INTAKING, DEPOSITING, OFF
    }

    public GroundIntake(HardwareMap hardwareMap)
    {
        intakeRunning=hardwareMap.get(CRServo.class, "intakeR");
        intakeSupporting = hardwareMap.get(CRServo.class, "intakeL");
        intakeRunning.setDirection(CRServo.Direction.REVERSE);
        setState(State.OFF);
    }

    public void update()
    {
        switch(state)
        {
            case INTAKING:
                intakeRunning.setPower(INTAKING);
                intakeSupporting.setPower(INTAKING);
                break;
            case DEPOSITING:
                intakeRunning.setPower(REVERSE);
                intakeSupporting.setPower(REVERSE);
                break;
            case OFF:
                intakeRunning.setPower(OFF);
                intakeSupporting.setPower(OFF);
                break;
        }
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
