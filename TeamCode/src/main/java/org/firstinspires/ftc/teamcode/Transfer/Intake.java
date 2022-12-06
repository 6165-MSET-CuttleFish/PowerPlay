package org.firstinspires.ftc.teamcode.Transfer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Module;

public class Intake extends Module
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

    public Intake(Robot r)
    {
        intakeRunning = r.hardwareMap.get(CRServo.class, "intakeRun");
        intakeSupporting = r.hardwareMap.get(CRServo.class, "intakeSup");
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
    }

}
