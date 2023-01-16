package org.firstinspires.ftc.teamcode.modules.slides;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BPIDFController;

@Config
public class Slides {
    public DcMotorEx slidesLeft, slidesRight;
    DigitalChannel slidesLimitSwitch;
    public static ElapsedTime time = new ElapsedTime();
    //slides is 17.5 inches tall
    boolean switchModified=false;
    boolean switchPressed=false;
    public static boolean slidesCheck = false;
    double output=0;
    double posAtZero=0;

    public static int HIGH = 2370; //old = 1850
    public static int HIGH_DROP = 2080; //old = 1650
    public static int MID = 1550; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    public static int MID_DROP = 1180;
    public static int LOW = 650; //in inches, low junction is 13.5 inches
    public static int LOW_DROP = 250;
    public static int INTAKE_AUTO =  125;
    public static double p = 0.01, i = 2, d = 0.0002;
    public static double kV = 0, kA = 0, kStatic = 0;
    public BPIDFController pidController = new BPIDFController(new PIDCoefficients(p, i, d), kV, kA, kStatic);
    public static final double TICKS_PER_INCH = 43.3935;
    public Slides.State state;
    public enum State{
        HIGH, HIGH_DROP, MID, MID_DROP, LOW, LOW_DROP, BOTTOM, MANUAL, INTAKE_AUTO, ZERO
    }
    public Slides(HardwareMap hardwareMap)
    {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        slidesLimitSwitch = hardwareMap.get(DigitalChannel.class, "slidesLimitSwitch");
        slidesRight.setDirection(DcMotorEx.Direction.REVERSE);
        slidesLeft.setDirection(DcMotorEx.Direction.REVERSE);
        slidesRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(){
        //checkLimit();
//        slidesLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
//        slidesRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
//        slidesLeft.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
//        slidesRight.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
        switch(state) {
            case HIGH:
                pidController.setTargetPosition(HIGH+posAtZero);
                break;
            case HIGH_DROP:
                pidController.setTargetPosition(HIGH_DROP+posAtZero);
                break;
            case MID:
                pidController.setTargetPosition(MID+posAtZero);
                break;
            case MID_DROP:
                pidController.setTargetPosition(MID_DROP+posAtZero);
                break;
            case LOW:
                pidController.setTargetPosition(LOW+posAtZero);
                break;
            case LOW_DROP:
                pidController.setTargetPosition(LOW_DROP+posAtZero);
                break;
            case INTAKE_AUTO:
                pidController.setTargetPosition(INTAKE_AUTO+posAtZero);
                break;
            case BOTTOM:
                pidController.setTargetPosition(0+posAtZero);
                break;
            case MANUAL:
//                slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
        output = pidController.update(slidesRight.getCurrentPosition());

        slidesRight.setPower(output);
        slidesLeft.setPower(output);
//        if (slidesLimitSwitch.getState()) {
//            slidesLeft.setPower(0);
//            slidesRight.setPower(0);
//        }
    }

    public double getOuput()
    {
        return output;
    }

    public double secondsSpentInState() {
        return time.seconds();
}
    public double millisecondsSpentInState() {
        return time.milliseconds();
    }
public void checkLimit()
{
    switchPressed=slidesLimitSwitch.getState();
    if(switchPressed)
    {
        posAtZero=slidesRight.getCurrentPosition();
    }
}
public void setPowerManual(double power)
{
    /*if(switchPressed&&power<0)
    {
        power = 0;
    }*/
    slidesLeft.setPower(power);
    slidesRight.setPower(power);
}

    public Slides.State getState() {
        return state;
    }
    public void setState(Slides.State state){
        this.state = state;
        time.reset();
        update();
    }
    public void up(){
        slidesLeft.setTargetPosition((int) HIGH);
        slidesRight.setTargetPosition((int) HIGH);
        slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setPower(1);
        slidesRight.setPower(1);
    }
    public void mid(){
        slidesLeft.setTargetPosition((int) MID);
        slidesRight.setTargetPosition((int) MID);
        slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setPower(1);
        slidesRight.setPower(1);
    }
    public void low(){
        slidesLeft.setTargetPosition((int) LOW);
        slidesRight.setTargetPosition((int) LOW);
        slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setPower(1);
        slidesRight.setPower(1);
    }
    public void down(){
        slidesLeft.setTargetPosition(0);
        slidesRight.setTargetPosition(0);
        slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesLeft.setPower(1);
        slidesRight.setPower(1);
    }



    public static double inchesToTicks(double inches) {
        return (inches * TICKS_PER_INCH);
    }

    public static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

}