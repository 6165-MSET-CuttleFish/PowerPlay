package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Module {
    public State state;
    public enum State{

    }
    public Module(HardwareMap hardwareMap){

    }
    public void update(){

    }
    public State getState() {
        return state;
    }
    public void setState(State state){
        this.state = state;
    }
}
