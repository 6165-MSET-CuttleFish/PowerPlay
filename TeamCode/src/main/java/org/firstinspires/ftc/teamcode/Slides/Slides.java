package org.firstinspires.ftc.teamcode.Slides;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Slides {
    public DcMotorEx slidesLeft;
    public DcMotorEx slidesRight;
    //slides is 17.5 inches tall
    static final double HIGH = 16; //in inches, 33.5 - 17.5 (high junction height - slides height)
    static final double MID = 6; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    static final double LOW = 0; //in inches, low junction is 13.5 inches
    public static double kp = 0.005, ki = 0, kd = 0.05, kf = 0;
    public static PIDFController pidController = new PIDFController(kp, ki, kd, kf);
    public static double TICKS_PER_INCH = 43.39;
    public Slides.State state;
    public enum State{
        HIGH, MID, LOW
    }
    public Slides(HardwareMap hardwareMap) {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");
        setState(State.LOW);
        pidController.setTolerance(inchesToTicks(0.2), 5);
    }

    public void update(){
        switch(state) {
            case HIGH:
                slidesLeft.setTargetPosition((int) inchesToTicks(21.5));
                slidesRight.setTargetPosition((int) inchesToTicks(21.5));
                break;
            case MID:
                slidesLeft.setTargetPosition((int) inchesToTicks(11.5));
                slidesRight.setTargetPosition((int) inchesToTicks(11.5));
                break;
            case LOW:
                slidesLeft.setTargetPosition((int) inchesToTicks(1.5));
                slidesRight.setTargetPosition((int) inchesToTicks(1.5));
                break;
        }
    }

    public Slides.State getState() {
        return state;
    }
    public void setState(Slides.State state){
        this.state = state;
    }
    public static double ticksToInches(double ticks) {
        return -ticks / TICKS_PER_INCH;
    }
    public static double inchesToTicks(double inches) {
        return -(inches * 43.39);
    }
}
