package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Turret2
{

    static final int LEFT_POS = 367, RIGHT_POS = -367, ZERO_POS = 0, MIDLEFT=210, MIDRIGHT=-210;
    double targetPos=0;
    double posAtZero=0;
    public DcMotorEx turretMotor;
    public TouchSensor magnetic;
    public Turret2.State state;
    public enum State
    {
        IDLE, LEFT, RIGHT, MIDLEFT, MIDRIGHT, ZERO
    }

    public Turret2(HardwareMap hardwareMap)
    {
        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magnetic = hardwareMap.get(TouchSensor.class, "MLS");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setState(State.IDLE);
    }

    public void update()
    {
        /*if(magnetic.isPressed())
        {
            posAtZero=turretMotor.getCurrentPosition();
        }*/
        updateTarget();
        turretMotor.setPower(motorPower());
    }


    public void setState(Turret2.State state)
    {
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.state = state;
    }

    private void updateTarget()
    {
        switch(state)
        {
            case IDLE:
                targetPos = turretMotor.getCurrentPosition()+posAtZero;
                break;
            case RIGHT:
                targetPos=RIGHT_POS+posAtZero;
                break;
            case LEFT:
                targetPos=LEFT_POS+posAtZero;
                break;
            case ZERO:
                targetPos=ZERO_POS+posAtZero;
                break;
            case MIDLEFT:
                targetPos=MIDLEFT+posAtZero;
                break;
            case MIDRIGHT:
                targetPos=MIDRIGHT+posAtZero;
                break;
        }
    }

    public double motorPower()
    {
        double error=targetPos-turretMotor.getCurrentPosition();
        double errorAbs=Math.abs(error);
        double sign=Math.signum(error);

        if(errorAbs<10)
        {
            setState(State.IDLE);
            return 0;
        }
        return Math.sqrt(errorAbs/650)*sign/1.3;
    }

    public boolean isBusy()
    {
        if(state==State.IDLE)
        {
            return false;
        }
        return true;
    }

    public Turret2.State getState() {
        return state;
    }
}
