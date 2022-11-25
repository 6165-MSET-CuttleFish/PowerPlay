package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Turret2
{

    static final int LEFT_POS = -1955, RIGHT_POS = 1996, ZERO_POS = 20, MIDLEFT=210, MIDRIGHT=-210;
    double targetPos=0;
    double posAtZero=0;
    public DcMotorEx turretMotor;
    public Encoder encoder;
    public TouchSensor magnetic;
    public Turret2.State state;
    public enum State
    {
        IDLE, LEFT, RIGHT, MIDLEFT, MIDRIGHT, ZERO
    }

    public Turret2(HardwareMap hardwareMap)
    {
        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");

        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magnetic = hardwareMap.get(TouchSensor.class, "MLS");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.state = state;
    }

    private void updateTarget()
    {
        switch(state)
        {
            case IDLE:
                targetPos = encoder.getCurrentPosition()+posAtZero;
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
        double error=(targetPos-encoder.getCurrentPosition())/4.4;
        double errorAbs=Math.abs(error);
        double sign=Math.signum(error);
        double motorOil=(Math.sqrt(errorAbs/650));

        /*if(errorAbs<15&&motorOil<0.08)
        {
            setState(State.IDLE);
            return 0;
        }*/
        //return sign*0.3;
        return motorOil*sign;
        //650
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
