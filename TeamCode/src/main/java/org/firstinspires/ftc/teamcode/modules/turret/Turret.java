package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.PIDCoeff;
import org.firstinspires.ftc.teamcode.util.PIDControl;

@Config
public class Turret
{
    public static double kp=0.0502;
    public static double ki=0.183;
    public static double kd=3.74;
    public static double iSumMax=43.2;
    public static double stabThresh=40;
    public static ElapsedTime time = new ElapsedTime();

    PIDControl controller;
    PIDCoeff coeff;

    static final int LEFT_POS = -2100, RIGHT_POS = 2100, ZERO_POS = 0, INIT=1020, BACK = 4200;
    public static double closePower = 0.17;
    public static double farPower = 0.65;
    double targetPos=0;
    double posAtZero=0;
    public DcMotorEx turretMotor;
    public Encoder encoder;
    public TouchSensor limit;
    public Turret.State state;

    public double motorOil=0;

    public enum State
    {
        IDLE, LEFT, RIGHT, ZERO, MANUAL, AUTOALIGN, INIT, BACK
    }

    public Turret(HardwareMap hardwareMap, boolean teleop)
    {
        coeff=new PIDCoeff(kp ,ki, kd, iSumMax, stabThresh);
        controller=new PIDControl(coeff);

        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");

        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));

//        limit = hardwareMap.get(TouchSensor.class, "limit");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if(teleop)
        {
            setState(State.ZERO);
        }
        else
        {
            setState(State.INIT);
        }
    }

    public void update(/*double time*/)
    {
        updateTarget();
        double sign=Math.signum(targetPos-encoder.getCurrentPosition());
        double errorAbs=Math.abs(targetPos-encoder.getCurrentPosition());
        //motorOil=controller.calculate(encoder.getCurrentPosition(), targetPos, time)/100;
        if(state!=State.MANUAL)
        {
            if(errorAbs<15)
            {
                motorOil=0;
            }
            else if(errorAbs<200)
            {
                motorOil=sign*closePower;
            }
            else if(errorAbs>200)
            {
                motorOil=sign*farPower;
            }
            turretMotor.setPower(motorOil);
        }
    }

    public double secondsSpentInState() {
        return time.seconds();
    }
    public double millisecondsSpentInState() {
        return time.milliseconds();
    }
    public void setState(Turret.State state) {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.state = state;
        updateTarget();
        time.reset();
    }

    private void updateTarget()
    {

//        if(limit.isPressed())
//        {
//            posAtZero=/*some value based on where limit switch clicks*/0;
//        }
        //if hall effect then reset pos at zero

        switch(state)
        {
            case MANUAL:
                targetPos = encoder.getCurrentPosition();
                break;
            case IDLE:
                targetPos = encoder.getCurrentPosition();
                break;
            case RIGHT:
                targetPos=RIGHT_POS+posAtZero;
                break;
            case LEFT:
                targetPos=LEFT_POS+posAtZero;
                break;
            case BACK:
                targetPos=BACK+posAtZero;
                break;
            case ZERO:
                targetPos=ZERO_POS+posAtZero;
                break;
            case INIT:
                targetPos=INIT+posAtZero;
                break;
            case AUTOALIGN:
                break;
        }
    }

    public boolean isBusy()
    {
        if(state==State.IDLE)
        {
            return false;
        }
        return true;
    }

    public Turret.State getState()
    {
        return state;
    }
}
