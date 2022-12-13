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
        IDLE(null), LEFT(LEFT_POS),
        RIGHT(RIGHT_POS), ZERO(ZERO_POS),
        MANUAL(null), AUTOALIGN(null),
        INIT(INIT_POS);

        private final Double position;
        State(Double position)
        {
            this.position=position;
        }
        @Override
        public Double getValue()
        {
            return position;
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

    private void updateTarget()
    {
        if(limit.isPressed())
            posAtZero=/*some value based on where limit switch clicks*/0;

        if(state.getValue()!=null)
            targetPos=state.getValue()+posAtZero;
        else if(state==State.AUTOALIGN)
            targetPos=/*pos got from auto align*/0+posAtZero;
    }

    @Override
    public void setState(ModuleState s)
    {
        w.interrupt();
        state=s;
        if(state!=State.MANUAL&&state!=State.IDLE)
        {
            w.start();
        }
    }

    @Override
    public void setState(ModuleState s, int delayMilis)
    {
        w.interrupt();
        state=s;
        if(state!=State.MANUAL&&state!=State.IDLE)
        {
            w.startDelay(delayMilis);
        }
    }

    public boolean isBusy()
    {
        if(state==State.IDLE)
            return false;

        return true;
    }
}
