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
    public CRServo groundLeft, groundRight;
    public DistanceSensor distSens;
    public State state;
    public enum State
    {
        INTAKING, DEPOSITING, OFF
    }

    public GroundIntake(HardwareMap hardwareMap)
    {

        groundLeft = hardwareMap.get(CRServo.class, "gl");
        groundRight = hardwareMap.get(CRServo.class, "gr");

        distSens = hardwareMap.get(DistanceSensor.class, "distanceG");
        groundRight.setDirection(CRServo.Direction.REVERSE);
        setState(State.OFF);
    }

    public void update()
    {
        switch(state)
        {
            case INTAKING:
                groundLeft.setPower(INTAKING);
                groundRight.setPower(INTAKING);
                break;
            case DEPOSITING:
                groundLeft.setPower(REVERSE);
                groundRight.setPower(REVERSE);
                break;
            case OFF:
                groundLeft.setPower(OFF);
                groundRight.setPower(OFF);
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
