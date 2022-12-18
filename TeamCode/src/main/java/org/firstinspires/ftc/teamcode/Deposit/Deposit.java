package org.firstinspires.ftc.teamcode.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
public class Deposit {
    //temporary values
    static double EXTENDED = 1;
    static double HALF = 0.5;
    static double ZERO = 0;
    Servo deposit1;
    Servo deposit2;
    public State state;
    public enum State
    {
        EXTEND, MIDDLE, RETRACT
    }

    public Deposit(HardwareMap hardwareMap) {
        deposit1=hardwareMap.get(Servo.class, "deposit1");
        deposit2 = hardwareMap.get(Servo.class, "deposit2");
        setState(State.RETRACT);
    }

    public void update()
    {
        switch(state) {
            case EXTEND:
                deposit1.setPosition(EXTENDED);
                deposit2.setPosition(EXTENDED);
                break;
            case MIDDLE:
                deposit1.setPosition(HALF);
                deposit2.setPosition(HALF);
                break;
            case RETRACT:
                deposit1.setPosition(ZERO);
                deposit2.setPosition(ZERO);
                break;
        }
    }

    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
        update();
    }

}
