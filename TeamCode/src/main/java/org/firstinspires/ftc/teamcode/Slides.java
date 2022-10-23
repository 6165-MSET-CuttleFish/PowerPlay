package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {

    DigitalChannel limitSwitch;
    public DcMotorEx slidesLeft;
    public DcMotorEx slidesRight;
    //slides is 17.5 inches tall
    static final double HIGH = 16; //in inches, 33.5 - 17.5 (high junction height - slides height)
    static final double MID = 6; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    static final double LOW = 0; //in inches, low junction is 13.5 inches
    static final double THRESHOLD=20;

    public targetPos target;
    public enum targetPos {
        HIGH, MID, LOW
    }

    public State state;
    public enum State
    {
        MOVING, STOPPED, RESETTING
    }

    public Slides(HardwareMap hardwareMap) {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        //reverse a motor
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        setTarget(targetPos.LOW);
    }

    public void update()
    {
        switch(state)
        {
            case MOVING:
                slidesLeft.setPower(motorPower());
                slidesRight.setPower(motorPower());
                if(Math.abs(((slidesLeft.getCurrentPosition()+slidesRight.getCurrentPosition())/2)- targetTicks())<THRESHOLD)
                {
                    state=State.STOPPED;
                }
                break;
            case RESETTING:
                slidesLeft.setPower(motorPower());
                slidesRight.setPower(motorPower());
                if(limitSwitch.getState()==true)
                {
                    state=State.STOPPED;
                }
                break;
            case STOPPED:
                //maybe continue running motors w/ PID loop? Depends on if it can't stay in place
                break;
        }
    }



    private double targetTicks()
    {
        switch(target)
        {
            case HIGH:
                return HIGH;
            case MID:
                return MID;
            case LOW:
                return LOW;
        }
        return LOW;
    }
    private double motorPower()
    {
        //use pid to determine w/ current and target position motor velocity
        return 0.5;
    }



    public void setTarget(targetPos target)
    {
        this.target = target;
        this.state=State.MOVING;
    }
    public void resetPos()
    {
        this.target=targetPos.LOW;
        this.state=State.RESETTING;
    }



    public targetPos getTarget()
    {
        return target;
    }
    public Slides.State getState()
    {
        return state;
    }
}