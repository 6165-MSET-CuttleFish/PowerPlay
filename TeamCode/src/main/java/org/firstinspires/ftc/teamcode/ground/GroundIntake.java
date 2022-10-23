package org.firstinspires.ftc.teamcode.ground;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class GroundIntake
{
    //temporary values
    static final double INTAKING = 1;
    static final double REVERSE = -1;
    static final double OFF = 0;
boolean runningTrigger = false;
boolean temp2 = false;
    CRServo intakeRunning;
    CRServo intakeSupporting;
    DistanceSensor distSens;
    public State state;
    public enum State
    {
        INTAKING, DEPOSITING, OFF
    }

    public GroundIntake(HardwareMap hardwareMap)
    {
        intakeRunning=hardwareMap.get(CRServo.class, "intakeR");
        intakeSupporting = hardwareMap.get(CRServo.class, "intakeL");
distSens = hardwareMap.get(DistanceSensor.class, "distanceG");
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
public boolean gSensor(){
        if(runningTrigger){
            runningTrigger = false;
        }
        else if(distSens.getDistance(DistanceUnit.MM)<22&& !temp2){
            runningTrigger = true;
            temp2 = true;
        }
        if(distSens.getDistance(DistanceUnit.MM)>50){
            temp2 = false;
        }
        return runningTrigger;

}
    public void setState(State state)
    {
        this.state = state;
        update();
    }

}
