package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.slides.Slides;

@Config
public class Deposit {
    //temporary values
    public static double LEXTENDED = 0.5;
    public static double REXTENDED = 0.22;
    public static double LZERO = 0.3;
    public static double RZERO = 0.01;
    public static double LVECTORING = 0.25;
    public static double RVECTORING = 0.26;
    public static double LINTAKE = 0.34;
    public static double RINTAKE = 0.33;
    public static double rightPos = 0;

    Servo leftExtension;
    Servo leftAngular;
    Servo rightExtension;
    Servo rightAngular;
    public ExtensionState extState=ExtensionState.RETRACT;
    public AngleState angState=AngleState.INTAKE;
    Slides slides;
    public enum ExtensionState
    {
        EXTEND, RETRACT
    }
    public enum AngleState
    {
        VECTORING, INTAKE
    }

    public Deposit(HardwareMap hardwareMap) {
        leftExtension =hardwareMap.get(Servo.class, "lExt");
        leftAngular = hardwareMap.get(Servo.class, "lAng");
        rightExtension =hardwareMap.get(Servo.class, "rExt");
        rightAngular= hardwareMap.get(Servo.class, "rAng");
        rightAngular.setDirection(Servo.Direction.REVERSE);
        rightExtension.setDirection(Servo.Direction.REVERSE);
        setExtension(ExtensionState.RETRACT);
        setAngle(AngleState.INTAKE);

    }

    public void update() {
        switch(extState) {
            case EXTEND:
                leftExtension.setPosition(LEXTENDED);
                rightExtension.setPosition(REXTENDED);
                break;
            case RETRACT:
                leftExtension.setPosition(LZERO);
                rightExtension.setPosition(RZERO);
                slides.setState(Slides.State.BOTTOM_RETRACTED);
                break;
        }
        switch (angState){
            case VECTORING:
                leftAngular.setPosition(LVECTORING);
                rightAngular.setPosition(RVECTORING);
                break;
            case INTAKE:
                leftAngular.setPosition(LINTAKE);
                rightAngular.setPosition(RINTAKE);
                break;
        }
        rightPos = rightExtension.getPosition();
    }

    public ExtensionState getExtState() {
        return extState;
    }
    public AngleState getAngState(){
        return angState;
    }

    public void setExtension(ExtensionState state) {
        this.extState = state;
        update();
    }
    public void setAngle(AngleState state){
        this.angState = state;
        update();
    }

}
