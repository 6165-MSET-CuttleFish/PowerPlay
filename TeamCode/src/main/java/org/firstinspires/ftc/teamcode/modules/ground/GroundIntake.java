package org.firstinspires.ftc.teamcode.modules.ground;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GroundIntake
{
    //temporary values
    static final double INTAKING = 1;
    static final double REVERSE = -1;
    static final double OFF = 0;
    boolean runningTrigger = false;
    boolean temp2 = false;
    DcMotor groundIntake;
    DistanceSensor distSens;
    public State state;
    public enum State
    {
        INTAKING, DEPOSITING, OFF
    }

    public GroundIntake(HardwareMap hardwareMap)
    {
        groundIntake = hardwareMap.get(DcMotor.class, "gi");
//        distSens = hardwareMap.get(DistanceSensor.class, "distanceG");
        setState(State.OFF);
    }

    public void update()
    {
        switch(state)
        {
            case INTAKING:
                groundIntake.setPower(INTAKING);
                break;
            case DEPOSITING:
                groundIntake.setPower(REVERSE);
                break;
            case OFF:
                groundIntake.setPower(OFF);
                break;
        }
    }

    public State getState() {
        return state;
    }
//    public boolean gSensor(){
//        if(runningTrigger){
//            runningTrigger = false;
//        }
//        else if(distSens.getDistance(DistanceUnit.MM)<22&& !temp2){
//            runningTrigger = true;
//            temp2 = true;
//        }
//        if(distSens.getDistance(DistanceUnit.MM)>50){
//            temp2 = false;
//        }
//        return runningTrigger;
//
//    }
    public boolean temp2e() {
        return temp2;
    }
//    public double sensorVal(){
//        return distSens.getDistance(DistanceUnit.MM);
//    }
    public boolean runningTriggere(){
        return runningTrigger;
    }
    public void setState(State state)
    {
        this.state = state;
        update();
    }

}
