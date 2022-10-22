package org.firstinspires.ftc.teamcode.Slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    public DcMotorEx slidesLeft;
    public DcMotorEx slidesRight;
    //slides is 17.5 inches tall
    static final double HIGH = 16; //in inches, 33.5 - 17.5 (high junction height - slides height)
    static final double MID = 6; //in inches, 23.5 - 17.5 (mid junction height - slides height)
    static final double LOW = 0; //in inches, low junction is 13.5 inches

    public Slides.State state;
    public enum State{
        HIGH, MID, LOW
    }
    public Slides(HardwareMap hardwareMap) {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "sl");
        slidesRight = hardwareMap.get(DcMotorEx.class, "sr");
        setState(State.LOW);
    }

    public void update(){
        switch(state) {
            case HIGH:
                //TODO:
            case MID:
                //TODO
            case LOW:
                //TODO;
        }
    }

    public Slides.State getState() {
        return state;
    }
    public void setState(Slides.State state){
        this.state = state;
    }
    public double ticksToInches(double ticks) {

        return ticks;
    }
}
