package org.firstinspires.ftc.teamcode.Slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Slides {
    DcMotorEx slidesLeft;
    DcMotorEx slidesRight;
    DigitalChannel slidesLimitSwitch;

    //slides is 17.5 inches tall
    static final double HIGH = 1800;
    static final double HIGH_DROP = 1500;
    static final double MID = 946; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    static final double LOW = 124; //in inches, low junction is 13.5 inches
    public static PIDFCoefficients SLIDES_PIDF = new PIDFCoefficients(1.502, 0, 0, 0);
    public static PIDFCoefficients VELOCITY_PIDF = new PIDFCoefficients(0.5, 2.27, 0.1, 2);
    public static final double TICKS_PER_INCH = 43.3935;
    public Slides.State state;
    public enum State{
        HIGH, HIGH_DROP, MID, LOW, BOTTOM
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
       // setState(State.BOTTOM);
        //   setState(State.BOTTOM);
    }

    public void update(){
        slidesLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
        slidesRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, SLIDES_PIDF);
        slidesLeft.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
        slidesRight.setVelocityPIDFCoefficients(VELOCITY_PIDF.p, VELOCITY_PIDF.i, VELOCITY_PIDF.d, VELOCITY_PIDF.f);
        switch(state) {
            case HIGH:
                slidesLeft.setTargetPosition((int) HIGH);
                slidesRight.setTargetPosition((int) HIGH);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case HIGH_DROP:
                slidesLeft.setTargetPosition((int) HIGH_DROP);
                slidesRight.setTargetPosition((int) HIGH_DROP);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case MID:
                slidesLeft.setTargetPosition((int) MID);
                slidesRight.setTargetPosition((int) MID);
                slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slidesLeft.setPower(1);
                slidesRight.setPower(1);
                break;
            case LOW:
                slidesLeft.setTargetPosition((int) LOW);
                slidesRight.setTargetPosition((int) LOW);
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
                while (slidesLimitSwitch.getState()) {
                    slidesLeft.setPower(-1);
                    slidesRight.setPower(-1);

                }
                    slidesLeft.setPower(0);
                    slidesRight.setPower(0);
                break;
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
        slidesLeft.setPower(-1);
        slidesRight.setPower(-1);
    }
    public static double inchesToTicks(double inches) {
        return (inches * TICKS_PER_INCH);
    }

    public static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

}