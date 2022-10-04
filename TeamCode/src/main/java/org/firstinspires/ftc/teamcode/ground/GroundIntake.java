package org.firstinspires.ftc.teamcode.ground;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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
        intakeRunning=hardwareMap.get(CRServo.class, "GroundLeft");
        intakeSupporting = hardwareMap.get(CRServo.class, "GroundRight");
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
            case DEPOSITING:
                intakeRunning.setPower(REVERSE);
                intakeSupporting.setPower(REVERSE);
            case OFF:
                intakeRunning.setPower(OFF);
                intakeSupporting.setPower(OFF);
        }
    }

    public State getState() {
        return state;
    }

    public void setState(GroundIntake.State state)
    {
        this.state = state;
        update();
    }

}
