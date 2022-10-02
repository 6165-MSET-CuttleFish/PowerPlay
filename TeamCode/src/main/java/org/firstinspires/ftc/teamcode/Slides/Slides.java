package org.firstinspires.ftc.teamcode.Slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    DcMotorEx slidesLeft;
    DcMotorEx slidesRight;

    static final double HIGH = 8;
    static final double MID = 6;
    static final double LOW = 4;
    static final double GROUND = 2;
    static final double INTAKE = 0;

    public Slides.State state;
    public enum State{
        HIGH, MID, LOW, GROUND, INTAKE
    }
    public Slides(HardwareMap hardwareMap) {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slidesRight = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        setState(State.INTAKE);
    }

    public void update(){
        switch(state) {
            case HIGH:
                //TODO:
            case MID:
                //TODO
            case LOW:
                //TODO:
            case GROUND:
                //TODO
            case INTAKE:
                //TODO:
        }
    }

    public Slides.State getState() {
        return state;
    }
    public void setState(Slides.State state){
        this.state = state;
    }
}
