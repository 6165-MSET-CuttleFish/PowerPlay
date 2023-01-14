package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.util.BPIDFController;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.PIDCoeff;
import org.firstinspires.ftc.teamcode.util.PIDControl;
import org.firstinspires.ftc.teamcode.*;
@Config
public class Turret
{
    public static double p = 0.004, i = 0.002, d = 0.00028727;
    public static double kV = 0, kA = 0, kStatic = 0;
    public BPIDFController pidController;

    public static double iSumMax=43.2;
    public static double stabThresh=40;
    public static ElapsedTime time = new ElapsedTime();

    PIDControl controller;
    PIDCoeff coeff;

    public static double offset=20.0;

    static final int LEFT_POS = -2100, RIGHT_POS = 2100, ZERO_POS = 0, INIT=1020, BACK = 4100;
    public static double closePower = 0.3;
    public static double farPower = 0.8;
    double targetPos=0;
    public static double posAtZero=0;
    double prevHall=0;
    public DcMotorEx turretMotor;
    public Encoder encoder;
    public AnalogInput hallEffect;
    public Turret.State state;

    public double motorOil=0;

    public enum State
    {
        IDLE, LEFT, RIGHT, ZERO, MANUAL, AUTOALIGN, INIT, BACK
    }

    public Turret(HardwareMap hardwareMap, boolean teleop)
    {
        pidController= new BPIDFController(new PIDCoefficients(p, i, d), kV, kA, kStatic);

        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");

        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));

        hallEffect = hardwareMap.get(AnalogInput.class, "hallEffect");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(/*double time*/)
    {
        updateTarget();

        //motorOil=controller.calculate(encoder.getCurrentPosition(), targetPos, time)/100;
        if(state!=State.MANUAL)
        {
            pidController.setTargetPosition(targetPos);
            turretMotor.setPower(pidController.update(encoder.getCurrentPosition()));
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
        if(hallEffect.getVoltage()-prevHall<-1.0){
            posAtZero= encoder.getCurrentPosition()+offset;
        }
        prevHall=hallEffect.getVoltage();
        switch(state)
        {
            case MANUAL:
                targetPos = encoder.getCurrentPosition();
                break;
            case IDLE:
                targetPos = encoder.getCurrentPosition();
                break;
            case RIGHT:
                targetPos=RIGHT_POS-posAtZero;
                break;
            case LEFT:
                targetPos=LEFT_POS-posAtZero;
                break;
            case BACK:
                targetPos=BACK-posAtZero;
                break;
            case ZERO:
                targetPos=ZERO_POS-posAtZero;
                break;
            case INIT:
                targetPos=INIT-posAtZero;
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