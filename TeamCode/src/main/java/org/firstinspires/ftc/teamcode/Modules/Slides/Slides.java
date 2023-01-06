package org.firstinspires.ftc.teamcode.Modules.Slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Turret.Turret;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.MotorModule;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.MotorWorker;
import org.firstinspires.ftc.teamcode.util.PIDCoeff;
import org.firstinspires.ftc.teamcode.util.PIDControl;

@Config
public class Slides extends MotorModule
{
    PIDCoeff coeff=new PIDCoeff(0.1, 0, 0, 0, 0);
    PIDControl controller=new PIDControl(coeff);

    public DcMotorEx slidesLeft, slidesRight;
    DigitalChannel slidesLimitSwitch;

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
        MANUAL(null), STOPPED(null);

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
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        motors.add(slidesLeft);
        motors.add(slidesRight);
        slidesLimitSwitch = hardwareMap.get(DigitalChannel.class, "slidesLimitSwitch");
        timer=new ElapsedTime();

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setState(State.BOTTOM);
        w=new MotorWorker(this);
        w.start();
    }

    public double motorPower()
    {
        double power=controller.calculate(currentPos(), targetPos, timer.milliseconds());
        if(Math.abs(power)<0.08)
            setState(Turret.State.STOPPED);

        if(state==State.MANUAL)
            return manualPower;
        else if(state==State.STOPPED)
            return 0;
        return power;
    }

    public boolean resetPressed() { return slidesLimitSwitch.getState();}

    public double currentPos() { return slidesLeft.getCurrentPosition();}
}