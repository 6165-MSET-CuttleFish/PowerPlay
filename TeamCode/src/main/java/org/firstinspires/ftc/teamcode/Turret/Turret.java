package org.firstinspires.ftc.teamcode.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.HardwareModule;
import org.firstinspires.ftc.teamcode.util.ModuleState;
import org.firstinspires.ftc.teamcode.util.PIDCoeff;
import org.firstinspires.ftc.teamcode.util.PIDControl;

@Config
public class Turret extends HardwareModule
{
    PIDControl controller;
    PIDCoeff coeff;

    static final double LEFT_POS = -2100, RIGHT_POS = 2100, ZERO_POS = 0, INIT_POS=1020;
    public static double closePower = 0.17;
    public static double farPower = 0.65;

    double targetPos;

    double posAtZero=0;
    public DcMotorEx turretMotor;
    public Encoder encoder;
    public TouchSensor limit;

    public double motorOil=0;

    public enum State implements ModuleState
    {
        IDLE(null, 0), LEFT(LEFT_POS, null),
        RIGHT(RIGHT_POS, null), ZERO(ZERO_POS, null),
        MANUAL(null, 0), AUTOALIGN(null,1),
        INIT(INIT_POS, null);

        private final Double position;
        private final Integer specialCode;
        State(Double position, Integer specialCode)
        {
            this.position=position;
            this.specialCode=specialCode;
        }
        @Override
        public Double getValue()
        {
            return position;
        }
        @Override
        public Integer specialCode()
        {
            return specialCode;
        }
    }

    public Turret(HardwareMap hardwareMap, boolean teleop)
    {
        super();
        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");
        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        limit = hardwareMap.get(TouchSensor.class, "limit");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(/*double time*/)
    {
        updateTarget();
        double sign=Math.signum(targetPos-encoder.getCurrentPosition());
        double errorAbs=Math.abs(targetPos-encoder.getCurrentPosition());
        //motorOil=controller.calculate(encoder.getCurrentPosition(), targetPos, time)/100;
        if(state!=State.MANUAL)
        {
            if(errorAbs<15) {
                motorOil = 0;
                state = State.IDLE;
            }
            else if(errorAbs<200)
                motorOil=sign*closePower;

            else if(errorAbs>200)
                motorOil=sign*farPower;

            turretMotor.setPower(motorOil);
        }
    }

    private void updateTarget()
    {
        if(limit.isPressed())
        {
            posAtZero=/*some value based on where limit switch clicks*/0;
        }

        if(state.getValue()!=null)
        {
            targetPos=state.getValue()+posAtZero;
        }
        else if(state.specialCode()==0)
        {
            targetPos=encoder.getCurrentPosition()+posAtZero;
        }
        else if(state.specialCode()==1)
        {
            targetPos=/*pos got from auto align*/0;
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
}
