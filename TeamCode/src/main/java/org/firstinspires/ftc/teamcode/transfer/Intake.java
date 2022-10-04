package org.firstinspires.ftc.teamcode.transfer;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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
    public Intake.State state;
    public enum State
    {
        INTAKING, DEPOSITING, OFF
    }

    public Intake(HardwareMap hardwareMap)
    {
        intakeRunning=hardwareMap.get(CRServo.class, "IntakeRunning");
        intakeSupporting = hardwareMap.get(CRServo.class, "IntakeSupporting");
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
                intakeRunning.setPower(DEPOSITING);
                intakeSupporting.setPower(DEPOSITING);
            case OFF:
                intakeRunning.setPower(OFF);
                intakeSupporting.setPower(OFF);
        }
    }

    public Intake.State getState() {
        return state;
    }

    public void setState(Intake.State state)
    {
        this.state = state;
        update();
    }

}
