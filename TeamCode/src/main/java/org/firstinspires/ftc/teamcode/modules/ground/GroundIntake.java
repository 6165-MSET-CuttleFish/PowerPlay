package org.firstinspires.ftc.teamcode.modules.ground;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;

public class GroundIntake extends HwModule
{
    //temporary values
    static final double INTAKING = .62;
    static final double REVERSE = -.65;
    static final double FAST = -1;
    static final double OFF = 0;
    boolean runningTrigger = false;
    boolean temp2 = false;
    DcMotor groundIntake;
    ColorRangeSensor distSens;
    public State state;
    boolean overrideState=true;

    @Override
    public void setState(ModuleState s)
    {
        if(s.getClass()==GroundIntake.State.class)
        {
            state=(State)s;
        }
        update();
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    public enum State implements ModuleState
    {
        INTAKING, DEPOSITING, FAST, OFF
    }

    public GroundIntake(HardwareMap hardwareMap)
    {
        groundIntake = hardwareMap.get(DcMotor.class, "gIntake");
        //distSens = hardwareMap.get(ColorRangeSensor.class, "gDist");
        groundIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        groundIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        groundIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setState(State.OFF);
    }
    //public double getDistance(){
        //return distSens.getDistance(DistanceUnit.INCH);
    //}
    public void update()
    {
        if(overrideState)
        {
            switch(state)
            {
                case INTAKING:
                    groundIntake.setPower(INTAKING);
                    break;
                case DEPOSITING:
                    groundIntake.setPower(REVERSE);
                    break;
                case FAST:
                    groundIntake.setPower(FAST);
                    break;
                case OFF:
                    groundIntake.setPower(OFF);
                    break;
            }
        }
        else
        {
            groundIntake.setPower(0);
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

    public void setOverrideState(boolean state)
    {
        overrideState=state;
        update();
    }
    public boolean getOverrideState()
    {
        return overrideState;
    }
}
