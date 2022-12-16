package org.firstinspires.ftc.teamcode.Slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MotorModule;
import org.firstinspires.ftc.teamcode.util.ModuleState;

@Config
public class Slides extends MotorModule
{
    //maybe we can change slides to our pid later idk

    public DcMotorEx slidesLeft, slidesRight;
    DigitalChannel slidesLimitSwitch;

    //slides is 17.5 inches tall
    boolean switchModified=false;
    boolean switchPressed=false;
    ModuleState oldState=State.BOTTOM;

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
        MANUAL(null){ @Override
        public DcMotor.RunMode runMode(){return DcMotor.RunMode.RUN_USING_ENCODER;} },
        ZERO(null){ @Override
        public DcMotor.RunMode runMode(){return DcMotor.RunMode.STOP_AND_RESET_ENCODER;} };

        private final Double position;
        State(Double position)
        {
            this.position=position;
        }

        @Override
        public Double getValue() {
            return position;
        }

        public DcMotor.RunMode runMode(){return DcMotor.RunMode.RUN_TO_POSITION;}

    }
    public Slides(HardwareMap hardwareMap)
    {
        super();
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        motors.add(slidesLeft);
        motors.add(slidesRight);
        slidesLimitSwitch = hardwareMap.get(DigitalChannel.class, "slidesLimitSwitch");
        setState(State.ZERO);
    }

    public void update()
    {
            slidesLeft.setTargetPosition(state.getValue().intValue());
            slidesRight.setTargetPosition(state.getValue().intValue());
            slidesLeft.setPower(1);
            slidesRight.setPower(1);
    }

    public void updateTarget()
{
    switchPressed=slidesLimitSwitch.getState();
    if(!switchPressed)
    {
        switchModified=true;
    }

    if(switchPressed&&switchModified)
    {
        setState(State.ZERO);
        switchModified=false;
    }
}

@Override public boolean isBusy()
{
    if(state==oldState||state==State.MANUAL||state==State.ZERO)
        return false;
    oldState = state;
    return true;
}
}