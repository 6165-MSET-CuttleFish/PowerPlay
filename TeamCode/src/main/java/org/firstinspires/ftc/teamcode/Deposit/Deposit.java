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
    Servo leftDeposit1;
    Servo leftDeposit2;
    Servo rightDeposit1;
    Servo rightDeposit2;
    public State state;
    public enum State
    {
        EXTEND, MIDDLE, RETRACT
    }

    public Deposit(HardwareMap hardwareMap) {
        leftDeposit1 =hardwareMap.get(Servo.class, "ld1");
        leftDeposit2 = hardwareMap.get(Servo.class, "ld2");
        rightDeposit1 =hardwareMap.get(Servo.class, "ld1");
        rightDeposit2 = hardwareMap.get(Servo.class, "ld2");
        setState(State.RETRACT);
    }

    public void update()
    {
        switch(state) {
            case EXTEND:
                leftDeposit1.setPosition(EXTENDED);
                leftDeposit2.setPosition(EXTENDED);
                rightDeposit1.setPosition(EXTENDED);
                rightDeposit2.setPosition(EXTENDED);
                break;
            case RETRACT:
                leftDeposit1.setPosition(ZERO);
                leftDeposit2.setPosition(ZERO);
                rightDeposit1.setPosition(ZERO);
                rightDeposit2.setPosition(ZERO);
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
