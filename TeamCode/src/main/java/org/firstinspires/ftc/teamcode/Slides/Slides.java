package org.firstinspires.ftc.teamcode.Slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.HardwareModule;
import org.firstinspires.ftc.teamcode.util.ModuleState;

@Config
public class Slides extends HardwareModule {
    public DcMotorEx slidesLeft, slidesRight;
    DigitalChannel slidesLimitSwitch;

    //slides is 17.5 inches tall
    boolean switchModified=false;
    boolean switchPressed=false;

    public static double HIGH = 1850; //old = 1850
    public static double HIGH_DROP = 2080; //old = 1650
    public static double MID = 1450; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    public static double MID_DROP = 1180;
    public static double LOW = 600; //in inches, low junction is 13.5 inches
    public static double LOW_DROP = 250;
    public static double INTAKE_AUTO =  125;
    public static double BOTTOM=0;

    public enum State implements ModuleState {
        HIGH(Slides.HIGH), HIGH_DROP(Slides.HIGH_DROP),
        MID(Slides.MID), MID_DROP(Slides.MID_DROP),
        LOW(Slides.LOW), LOW_DROP(Slides.LOW_DROP),
        BOTTOM(Slides.BOTTOM),  INTAKE_AUTO(Slides.INTAKE_AUTO),
        MANUAL(null), ZERO(null);

        private final Double position;
        State(Double position)
        {
            this.position=position;
        }

        @Override
        public Double getValue() {
            return position;
        }
    }
    public Slides(HardwareMap hardwareMap)
    {
        super();
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        slidesLimitSwitch = hardwareMap.get(DigitalChannel.class, "slidesLimitSwitch");
        //slidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(){
        checkLimit();

        if(state.getValue()!=null)
        {
            slidesLeft.setTargetPosition(state.getValue().intValue());
            slidesRight.setTargetPosition(state.getValue().intValue());
            slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slidesLeft.setPower(1);
            slidesRight.setPower(1);
        }
        else if(state==State.MANUAL)
        {
            slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(state==State.ZERO)
        {
            slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesLeft.setPower(0);
            slidesRight.setPower(0);
        }
    }


public void checkLimit()
{
    switchPressed=slidesLimitSwitch.getState();
    if(!switchPressed)
    {
        switchModified=true;
    }


    if(switchPressed&&state==State.BOTTOM)
    {
        setState(State.ZERO);
    }
    else if(switchPressed&&state==State.MANUAL&&switchModified)
    {
        switchModified=false;
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

@Override public boolean isBusy()
{
    if(state==State.MANUAL||state==State.BOTTOM)
        return true;
    return false;
}
}