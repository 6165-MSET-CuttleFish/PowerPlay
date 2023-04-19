package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BPIDFController;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.TBHController;
import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;

@Config
public class Turret extends HwModule
{


    public static double p = 0.0021, i = 0.0012, d = 0.00024;
    PIDCoefficients coeff1=new PIDCoefficients(p, i, d);
    public static double p2=0.0023, i2=0.00094, d2=0.00021;
    PIDCoefficients coeff2=new PIDCoefficients(p2, i2, d2);

    public BPIDFController pidController;
    public TBHController autoalignController;

    public static ElapsedTime time = new ElapsedTime();

    public static int LEFT_POS = 2100, RIGHT_POS = -2100, ZERO_POS = 0, INIT=1020,
            BACK = 4125, RIGHT_DIAGONAL = -3000, LEFT_DIAGONAL = 3000,  RIGHT_SIDE_HIGH = -3075,
            RIGHT_SIDE_HIGH_SAFE = -3000,
            RIGHT_SIDE_HIGH_PRELOAD = -1030, RIGHT_SIDE_MID_PRELOAD = -3200, RIGHT_SIDE_MID = 3200,
            LEFT_SIDE_HIGH_PRELOAD = 1030, LEFT_SIDE_HIGH = 3075,LEFT_SIDE_MID = -3100,LEFT_SIDE_MID_PRELOAD = 3200;


    double targetPos=0;
    public double posAtZero=0;
    public DcMotorEx turretMotor;
    public Encoder encoder;
    public AnalogInput hallEffect;
    public Autoalign autoalign;
    public Turret.State state;

    public double factor=-2.5;
    double pastVel;

    HardwareMap hardwareMap;

    public static double gain=0.001;

    @Override
    public void setState(ModuleState s)
    {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(s.getClass()==Turret.State.class)
        {
            state=(State)s;
        }
        if(Context.autoalignCameraPastInit)
        {
            if(s==State.AUTOALIGN)
            {
                autoalign.alignstate=true;
            }
            else
            {
                autoalign.alignstate = false;
            }
        }
        updateTarget();
    }
    public enum State implements ModuleState
    {
        IDLE, LEFT, RIGHT, ZERO, MANUAL, AUTOALIGN, INIT, BACK,
        RIGHT_SIDE_HIGH, RIGHT_SIDE_HIGH_PRELOAD, RIGHT_DIAGONAL,
        LEFT_DIAGONAL, RIGHT_SIDE_MID, RIGHT_SIDE_MID_PRELOAD, LEFT_SIDE_HIGH, LEFT_SIDE_HIGH_PRELOAD,
        LEFT_SIDE_MID, LEFT_SIDE_MID_PRELOAD, Right_SIDE_MID, Right_SIDE_MID_PRELOAD, RIGHT_SIDE_HIGH_SAFE
    }

    public Turret(HardwareMap hardwareMap)
    {
        this.hardwareMap=hardwareMap;
        pidController=new BPIDFController(coeff1);
        autoalignController=new TBHController(gain);

        turretMotor=hardwareMap.get(DcMotorEx.class, "hturret");

        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        //voltSensor=hardwareMap.voltageSensor.get("hturret");
        hallEffect =hardwareMap.get(AnalogInput.class, "hallEffect");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        state=State.ZERO;
    }

    public void update() {
        updateTarget();

        /*if(voltSensor.getVoltage()>13)
        {

        }
        else
        {

        }*/

        if(Math.abs(encoder.getCurrentPosition()) < 600) {
            pidController.gainSchedule(coeff2);
        }
        else {
            pidController.gainSchedule(coeff1);
        }

        if(state==State.AUTOALIGN&&Context.autoalignCameraPastInit)
        {
            //autoalignController.setTargetVel(autoalign.getVel());
            //turretMotor.setPower(autoalignController.update(encoder.getCorrectedVelocity()));

            //turretMotor.setPower(autoalign.getPower());
            //turretMotor.setPower(pidController.update(encoder.getCurrentPosition()));
            //pidController.setTargetPosition(targetPos);
            //turretMotor.setPower(pidController.update(encoder.getCurrentPosition()));

            if(!Context.autoalignConstantSpeed)
            {
                pidController.setTargetPosition(targetPos);
                turretMotor.setPower(pidController.update(encoder.getCurrentPosition()));
            }
            else
            {
                turretMotor.setPower(autoalign.getPower());
            }
        }
        else if(state!=State.MANUAL) {
            pidController.setTargetPosition(targetPos);
            turretMotor.setPower(pidController.update(encoder.getCurrentPosition()));
            //autoalignController.setTargetVel(0);
        }
    }

    public double secondsSpentInState() {
        return time.seconds();
    }
    public double millisecondsSpentInState() {
        return time.milliseconds();
    }

    private void updateTarget() {
//        if(Context.hallEffectEnabled){
//            if (hallEffect.getVoltage()<1.0) {
//                posAtZero = -encoder.getCurrentPosition();
//            }
//        }
            switch (state) {
                case MANUAL:
                case IDLE:
                    targetPos = encoder.getCurrentPosition();
                    break;
                case RIGHT:
                    targetPos = RIGHT_POS - posAtZero;
                    break;
                case LEFT:
                    targetPos = LEFT_POS - posAtZero;
                    break;
                case BACK:
                    targetPos = BACK - posAtZero;
                    break;
                case ZERO:
                    targetPos = ZERO_POS - posAtZero;
                    break;
                case RIGHT_SIDE_HIGH:
                    targetPos = RIGHT_SIDE_HIGH - posAtZero;
                    break;
                case RIGHT_SIDE_HIGH_PRELOAD:
                    targetPos = RIGHT_SIDE_HIGH_PRELOAD - posAtZero;
                    break;
                case LEFT_SIDE_HIGH:
                    targetPos = LEFT_SIDE_HIGH - posAtZero;
                    break;
                case LEFT_SIDE_HIGH_PRELOAD:
                    targetPos = LEFT_SIDE_HIGH_PRELOAD - posAtZero;
                    break;
                case LEFT_SIDE_MID_PRELOAD:
                    targetPos = LEFT_SIDE_MID_PRELOAD - posAtZero;
                    break;
                case RIGHT_SIDE_MID_PRELOAD:
                    targetPos = RIGHT_SIDE_MID_PRELOAD - posAtZero;
                    break;
                case RIGHT_SIDE_MID:
                    targetPos = RIGHT_SIDE_MID - posAtZero;
                    break;
                case LEFT_SIDE_MID:
                    targetPos = LEFT_SIDE_MID - posAtZero;
                    break;
                case INIT:
                    targetPos = INIT - posAtZero;
                    break;
                case RIGHT_SIDE_HIGH_SAFE:
                    targetPos = RIGHT_SIDE_HIGH_SAFE - posAtZero;
                    break;
                case RIGHT_DIAGONAL:
                    targetPos = RIGHT_DIAGONAL - posAtZero;
                    break;
                case LEFT_DIAGONAL:
                    targetPos = LEFT_DIAGONAL - posAtZero;
                    break;
                case AUTOALIGN:
                    if(Context.autoalignCameraPastInit)
                    {
                        if(autoalign.centerX>-1)
                        {
                            targetPos=encoder.getCurrentPosition()+(autoalign.centerX-160)*factor*(13/getBatteryVoltage());
                        }
                        else
                        {
                            targetPos=encoder.getCurrentPosition();
                        }
                        //targetPos = encoder.getCurrentPosition()+autoalign.getShift();
                    }
                    else
                    {
                        targetPos=encoder.getCurrentPosition();
                    }
                    break;
            }
    }

    public void setAutoalignCamera(Autoalign autoalign)
    {
        this.autoalign=autoalign;
    }

    public void resetPos() {
        posAtZero = encoder.getCurrentPosition();
    }
    public double getTargetPos() {
        return targetPos;
    }
    public boolean isBusy()
    {
        if(state==State.IDLE)
        {
            return false;
        }
        return true;
    }

    public double getBatteryVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
            {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public Turret.State getState()
    {
        return state;
    }
}