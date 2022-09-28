package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Turret extends Module{
    public State state;
    private DcMotor turret;

    public abstract void setPower(float v);

    public enum State{
        IDLE,
        ROTATING_LEFT,
        ROTATING_RIGHT,
    }
    public Turret(HardwareMap hardwareMap){
        super(hardwareMap);
    }
    public void update(){
        switch (getState()) {
            case IDLE:
                break;
            case ROTATING_LEFT:
                turret.setPower(-1.0);
                break;
            case ROTATING_RIGHT:
                turret.setPower(1.0);
                break;
        }

    }
    public State getState() {
        return state;
    }
    public void setState(State state){
        this.state = state;
    }
    public void leftRotate() {
        setState(State.ROTATING_LEFT);
    }

    public void idle() {
        setState(State.IDLE);
    }

    public void rightRotate() {
        setState(State.ROTATING_RIGHT);
    }
}
