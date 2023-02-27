package org.firstinspires.ftc.teamcode.modules.slides;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BPIDFController;
import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;

@Config
public class Slides extends HwModule {
    public DcMotorEx slidesLeft, slidesRight;
    //public AnalogInput slidesLimitSwitch;
    //public DigitalChannel slidesLimitSwitch;
    public static ElapsedTime time = new ElapsedTime();
    //slides is 17.5 inches tall
    boolean switchModified=false;
    double switchPressed;
    public static boolean slidesCheck = false;
    double output=0;
    public double posAtZero=0;
    public double manual = 0;

    public static int HIGH = 2380; //old = 1850
    public static int CYCLE_HIGH = 2325; //old = 1650
    public static int MID = 1590; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    public static int MID_DROP = 1180;
    public static int LOW = 860; //in inches, low junction is 13.5 inches
    public static int LOW_DROP = 250;
    public static int PICKUP = 10;
    public static int INTAKE_AUTO =  125;
    public static int SLIGHT = 475;
    public static int CYCLE0 = 380;
    public static int CYCLE1 = 285;
    public static int CYCLE2 = 190;
    public static int CYCLE3 = 95;

    public static int CYCLE4 = 0;

    public static double p = 0.01, i = 2, d = 0.0002;
    public static double kV = 0, kA = 0, kStatic = 0;
    public BPIDFController pidController = new BPIDFController(new PIDCoefficients(p, i, d), kV, kA, kStatic);
    public static final double TICKS_PER_INCH = 43.3935;
    public Slides.State state;

    @Override
    public void setState(ModuleState s) {
        if(s.getClass()==Slides.State.class)
        {
            state=(State) s;
        }
        time.reset();
        update();
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    public enum State implements ModuleState {
        HIGH, CYCLE_HIGH, MID, MID_DROP, LOW, LOW_DROP,
        BOTTOM, MANUAL, INTAKE_AUTO, ZERO, PICKUP,
        CYCLE0,CYCLE1,CYCLE2,CYCLE3,CYCLE4, SLIGHT
    }
    public Slides(HardwareMap hardwareMap) {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        //slidesLimitSwitch = hardwareMap.get(DigitalChannel.class, "sLimit");
        slidesLeft.setDirection(DcMotorEx.Direction.REVERSE);
        slidesRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        state=State.BOTTOM;
    }

    public void update() {
        //checkLimit();
//        slidesLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
//        slidesRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
//        slidesLeft.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
//        slidesRight.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
        switch (state) {
            case HIGH:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(HIGH + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case CYCLE_HIGH:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(CYCLE_HIGH + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case MID:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(MID + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case MID_DROP:
                pidController.setTargetPosition(MID_DROP + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case LOW:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(LOW + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case LOW_DROP:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(LOW_DROP + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case INTAKE_AUTO:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(INTAKE_AUTO + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case BOTTOM:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(0 + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case PICKUP:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(PICKUP + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case CYCLE0:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(CYCLE0 + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case CYCLE1:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(CYCLE1 + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case CYCLE2:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(CYCLE2 + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case CYCLE3:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(CYCLE3 + posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case CYCLE4:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pidController.setTargetPosition(posAtZero);
                output = pidController.update(slidesRight.getCurrentPosition());
                slidesRight.setPower(output);
                slidesLeft.setPower(output);
                break;
            case MANUAL:
                slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                output = pidController.update(slidesRight.getCurrentPosition());
                if (Math.abs(manual) > 0.1) {
                    slidesRight.setPower(manual);
                    slidesLeft.setPower(manual);
                    pidController.setTargetPosition(slidesRight.getCurrentPosition());
                } else {
                    slidesRight.setPower(output);
                    slidesLeft.setPower(output);
                }
                break;
            case SLIGHT:
                pidController.setTargetPosition(SLIGHT+posAtZero);
                break;
        }
//        output = pidController.update(slidesRight.getCurrentPosition());
//
//        slidesRight.setPower(output);
//        slidesLeft.setPower(output);
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
    public void checkLimit() {
        /*if (limitPressed()) {
            posAtZero = slidesRight.getCurrentPosition();
        }*/
    }
    public void setPowerManual(double power) {
        manual = power;
    }

    /*public boolean limitPressed() {
        return !slidesLimitSwitch.getState();
    }*/

    public Slides.State getState() {
        return state;
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