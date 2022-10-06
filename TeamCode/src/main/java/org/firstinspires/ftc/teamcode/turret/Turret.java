package org.firstinspires.ftc.teamcode.turret;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Module;

public class Turret {
    public Turret.State state;
    private DcMotor turret;
    public enum State {
        IDLE,
        ROTATING_LEFT,
        ROTATING_RIGHT,
    }
    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turret");
        setState(State.IDLE);
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
    public Turret.State getState() {
        return this.state;
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
