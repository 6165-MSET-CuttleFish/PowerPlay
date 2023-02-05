package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.slides.Slides;
//SLIDES, TURRET, HORIZONTAL EXT (probably tmr), COMBINED
@Config
public class Deposit {
    //temporary values
    public static double LEXTENDED = 0.5;
    public static double REXTENDED = 0.22;
    public static double LHALF = 0.38;
    public static double RHALF = 0.2;
    public static double LZERO = 0.3;
    public static double RZERO = 0.01;
    public static double rightPos = 0;

    public static double VECTORING = 0.46;
    public static double INTAKE = 0.65;
    public static double PICKUP = 1;

    Servo leftExtension;
    Servo rightExtension;
    Servo wrist;
    public ExtensionState extState = ExtensionState.RETRACT;
    public AngleState angState=AngleState.INTAKE;
    Slides slides;
    public enum ExtensionState
    {
        EXTEND, RETRACT, SLIGHT, HALF
    }
    public enum AngleState
    {
        VECTORING, INTAKE, CONE_PICKUP, X
    }

    public Deposit(HardwareMap hardwareMap) {
//        leftExtension =hardwareMap.get(Servo.class, "lExt");
//        rightExtension =hardwareMap.get(Servo.class, "rExt");
        wrist = hardwareMap.get(Servo.class, "wrist");
//        rightExtension.setDirection(Servo.Direction.REVERSE);
//        setExtension(ExtensionState.RETRACT);
        setAngle(AngleState.INTAKE);
    }

    public void update() {
        switch(extState) {
            case EXTEND:
//                leftExtension.setPosition(LEXTENDED);
//                rightExtension.setPosition(REXTENDED);
                break;
            case RETRACT:
//                leftExtension.setPosition(LZERO);
//                rightExtension.setPosition(RZERO);
                break;
            case HALF:
//                leftExtension.setPosition(LHALF);
//                rightExtension.setPosition(RHALF);
                break;
        }
        switch (angState){
            case VECTORING:
                wrist.setPosition(VECTORING);
                break;
            case INTAKE:
                wrist.setPosition(INTAKE);
                break;
            case CONE_PICKUP:
                wrist.setPosition(PICKUP);
            case X:
                break;
        }
//        rightPos = rightExtension.getPosition();
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
