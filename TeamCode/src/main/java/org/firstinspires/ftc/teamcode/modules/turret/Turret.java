package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BPIDFController;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;

@Config
public class Turret extends HwModule
{
    public static double p = 0.0027, i = 0.0012, d = 0.00024;
    PIDCoefficients coeff1=new PIDCoefficients(p, i, d);
    public static double p2=0.0023, i2=0.00094, d2=0.00021;
    PIDCoefficients coeff2=new PIDCoefficients(p2, i2, d2);

    public BPIDFController pidController;

    public static ElapsedTime time = new ElapsedTime();

    public static double offset=8;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    public static int LEFT_POS = 2100, RIGHT_POS = -2100, ZERO_POS = 0, INIT=1020,
            BACK = 4125, RIGHT_DIAGONAL = -3000, LEFT_DIAGONAL = 3000,  RIGHT_SIDE_HIGH = -3025,
            RIGHT_SIDE_HIGH_PRELOAD = -1030, RIGHT_SIDE_MID_PRELOAD = -3200, RIGHT_SIDE_MID = 3200,
            LEFT_SIDE_HIGH_PRELOAD = 1030, LEFT_SIDE_HIGH = 2970,LEFT_SIDE_MID = -3100,LEFT_SIDE_MID_PRELOAD = 3200;



    public static double closePower = 0.3;
    public static double farPower = 0.8; //:)
    double targetPos=0;
    public double posAtZero=0;
    public double prevHall=0;
    public DcMotorEx turretMotor;
    public Encoder encoder;
    public AnalogInput hallEffect;
    public AlignerAuto autoalign;
    public Turret.State state;
    public double motorOil=0;
    @Override
    public void setState(ModuleState s)
    {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(s.getClass()==Turret.State.class)
        {
            state=(State)s;
        }
        if(s==State.AUTOALIGN)
        {
            autoalign.alignstate=true;
        }
        else
        {
            autoalign.alignstate = false;
        }
        updateTarget();
    }
    public enum State implements ModuleState
    {
        IDLE, LEFT, RIGHT, ZERO, MANUAL, AUTOALIGN, INIT, BACK,
        RIGHT_SIDE_HIGH, RIGHT_SIDE_HIGH_PRELOAD, RIGHT_DIAGONAL,
        LEFT_DIAGONAL, RIGHT_SIDE_MID, RIGHT_SIDE_MID_PRELOAD, LEFT_SIDE_HIGH, LEFT_SIDE_HIGH_PRELOAD,
        LEFT_SIDE_MID, LEFT_SIDE_MID_PRELOAD, Right_SIDE_MID, Right_SIDE_MID_PRELOAD
    }

    public Turret(HardwareMap hardwareMap)
    {
        pidController=new BPIDFController(coeff1);

        turretMotor=hardwareMap.get(DcMotorEx.class, "hturret");

        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        hallEffect =hardwareMap.get(AnalogInput.class, "hallEffect");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        state=State.ZERO;
    }

    public void update() {
        updateTarget();

        //motorOil=controller.calculate(encoder.getCurrentPosition(), targetPos, time)/100;

        if(Math.abs(encoder.getCurrentPosition()) < 600) {
            pidController.gainSchedule(coeff2);
        }
        else {
            pidController.gainSchedule(coeff1);
        }

        if(state==State.AUTOALIGN&&Context.autoalignCameraPastInit)
        {
           turretMotor.setPower(autoalign.getPower());
        }
        else if(state!=State.MANUAL) {
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

    private void updateTarget() {
//        if(Context.hallEffectEnabled){
//            if (hallEffect.getVoltage()<1.0) {
//                posAtZero = -encoder.getCurrentPosition();
//            }
//        }
            switch (state) {
                case MANUAL:
                case AUTOALIGN:
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
                case RIGHT_DIAGONAL:
                    targetPos = RIGHT_DIAGONAL - posAtZero;
                    break;
                case LEFT_DIAGONAL:
                    targetPos = LEFT_DIAGONAL - posAtZero;
                    break;
            }
    }

    public void setAutoalignCamera(AlignerAuto autoalign)
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

    public Turret.State getState()
    {
        return state;
    }
}