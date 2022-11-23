package org.firstinspires.ftc.teamcode.Slides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Slides {
    public DcMotorEx slidesLeft, slidesRight;
    DigitalChannel slidesLimitSwitch;

    //slides is 17.5 inches tall
    public static int HIGH = 2080; //old = 1850
    static final int HIGH_DROP = 2080; //old = 1650
    static final int MID = 1680; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    static final int MID_DROP = 950;
    static final int LOW = 625; //in inches, low junction is 13.5 inches
    static final int LOW_DROP = 250;
    public static PIDFCoefficients SLIDES_PIDF = new PIDFCoefficients(1.502, 0, 0, 0);
    public static PIDFCoefficients VELOCITY_PIDF = new PIDFCoefficients(2.5, 2.43, .075, .025);
    public static final double TICKS_PER_INCH = 43.3935;
    public Slides.State state;
    public enum State{
        HIGH, HIGH_DROP, MID, MID_DROP, LOW, LOW_DROP, BOTTOM, MANUAL
    }
    public Slides(HardwareMap hardwareMap) {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        slidesLimitSwitch = hardwareMap.get(DigitalChannel.class, "slidesLimitSwitch");
        slidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void update(){
//        slidesLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
//        slidesRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
//        slidesLeft.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
//        slidesRight.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
        switch(state) {
            case HIGH:
                slidesLeft.setTargetPosition(HIGH);
                slidesRight.setTargetPosition(HIGH);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case HIGH_DROP:
                slidesLeft.setTargetPosition(HIGH_DROP);
                slidesRight.setTargetPosition(HIGH_DROP);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case MID:
                slidesLeft.setTargetPosition(MID);
                slidesRight.setTargetPosition(MID);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case MID_DROP:
                slidesLeft.setTargetPosition(MID_DROP);
                slidesRight.setTargetPosition(MID_DROP);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case LOW:
                slidesLeft.setTargetPosition(LOW);
                slidesRight.setTargetPosition(LOW);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case LOW_DROP:
                slidesLeft.setTargetPosition(LOW_DROP);
                slidesRight.setTargetPosition(LOW_DROP);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case BOTTOM:
                slidesLeft.setTargetPosition(0);
                slidesRight.setTargetPosition(0);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case MANUAL:
                slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }
//        if (slidesLimitSwitch.getState()) {
//            slidesLeft.setPower(0);
//            slidesRight.setPower(0);
//        }
    }
public void resetcheck(){
        if(slidesLimitSwitch.getState()){
            slidesLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
            slidesRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
}
    public Slides.State getState() {
        return state;
    }
    public void setState(Slides.State state){
        this.state = state;
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