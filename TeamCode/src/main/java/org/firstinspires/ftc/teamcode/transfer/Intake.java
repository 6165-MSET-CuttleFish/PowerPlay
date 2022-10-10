package org.firstinspires.ftc.teamcode.transfer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake
{
    //temporary values
    static final double INTAKING = 1;
    static final double DEPOSITING = -1;
    static final double OFF = 0;

    CRServo intakeRunning;
    CRServo intakeSupporting;
    public State state;
    public enum State
    {
        INTAKING, DEPOSITING, OFF
    }

    public Intake(HardwareMap hardwareMap)
    {
        intakeRunning=hardwareMap.get(CRServo.class, "intakeRun");
        intakeSupporting = hardwareMap.get(CRServo.class, "intakeSup");
        intakeSupporting.setDirection(CRServo.Direction.REVERSE);
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
                intakeRunning.setPower(DEPOSITING);
                intakeSupporting.setPower(DEPOSITING);
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
