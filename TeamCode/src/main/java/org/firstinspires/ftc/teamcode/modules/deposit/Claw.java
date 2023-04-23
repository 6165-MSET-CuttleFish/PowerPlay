package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;

@Config
public class Claw extends HwModule {
    public static double OPEN_WIDE = 0.69;
    public static double OPEN = 0.73;
    public static double CLOSE = 1.0;
    public static double PARTIAL=0;

    Servo claw, pole;
    public State state = State.OPEN;
    public Pole poleState = Pole.UP;
    @Override
    public void setState(ModuleState s) {
        if(s.getClass()==Claw.State.class) {
            state=(State)s;
        }
        update();
    }
    public void setPoleState(ModuleState s) {
        if(s.getClass()==Claw.Pole.class) {
            poleState=(Pole)s;
        }
        update();
    }
    @Override
    public boolean isBusy() {
        return false;
    }


    public static double UP = 0.225, DOWN = 1, DEPOSIT = 0.794, SAFE = 0.795;
    public static double TELE_UP = 0.26, TELE_DOWN = 0.79, TELE_DEPOSIT = 0.95;
    public enum Pole implements ModuleState {
        UP, DOWN, DEPOSIT, SAFE,
        TELE_UP, TELE_DOWN, TELE_DEPOSIT
    }
    public enum State implements ModuleState {
        OPEN, CLOSE, PARTIAL, OPEN_WIDE
    }

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        pole = hardwareMap.get(Servo.class, "pole");
        setState(State.OPEN);
        setPoleState(Pole.UP);
    }

    public void update() {
        switch(poleState) {
            case UP:
                pole.setPosition(UP);
                break;
            case DOWN:
                pole.setPosition(DOWN);
                break;
            case DEPOSIT:
                pole.setPosition(DEPOSIT);
                break;
            case SAFE:
                pole.setPosition(SAFE);
                break;
            case TELE_UP:
                pole.setPosition(TELE_UP);
                break;
            case TELE_DOWN:
                pole.setPosition(TELE_DOWN);
                break;
            case TELE_DEPOSIT:
                pole.setPosition(TELE_DEPOSIT);
                break;
        }

        switch(state) {
            case OPEN:
                claw.setPosition(OPEN);
                break;
            case CLOSE:
                claw.setPosition(CLOSE);
                break;
            case PARTIAL:
                claw.setPosition(PARTIAL);
                break;
            case OPEN_WIDE:
                claw.setPosition(OPEN_WIDE);
                break;
        }
    }

    public Claw.State getState() {
        return state;
    }

    public Claw.Pole getPoleState() {
        return poleState;
    }
}
