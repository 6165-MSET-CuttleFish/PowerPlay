package org.firstinspires.ftc.teamcode.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.easyopencv.OpenCvWebcam;

public class Turret
{
    static final double     COUNTS_PER_MOTOR_REV    = 65;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 8.26771654;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TURN_SPEED             = 1;
    static final int LEFT_POS = 366, RIGHT_POS = -366, ZERO_POS = 0;
    double endPosition;
    public DcMotorEx turretMotor;
    Detector detector1;
    OpenCvWebcam webcam;
    public TouchSensor magnetic;
    public Turret.State state;
    public int prevPositionReset = 0, position = 0;
    public PIDController pidController;
    public enum State {
        IDLE, MOVING, LEFT, RIGHT, ZERO, MANUAL
    }

    public Turret(HardwareMap hardwareMap)
    {
        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magnetic = hardwareMap.get(TouchSensor.class, "MLS");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setState(State.IDLE);
    }

    public void update()
    {
        switch(state)
        {
            case MOVING:
                break;
            case IDLE:
                turretMotor.setPower(0);
                break;
            case RIGHT:
                turretMotor.setTargetPositionTolerance(5);
                turretMotor.setTargetPosition(RIGHT_POS);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.5);
                break;
            case LEFT:
                turretMotor.setTargetPositionTolerance(5);
                turretMotor.setTargetPosition(LEFT_POS);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.5);
                break;
            case ZERO:
                turretMotor.setTargetPositionTolerance(5);
                turretMotor.setTargetPosition(ZERO_POS);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.93);
                break;
//            case MANUAL:
//                turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                break;

        }
    }

    public Turret.State getState() {
        return state;
    }

    public void setState(Turret.State state)
    {
        this.state = state;
        update();
    }

    public void autoAlign(){
        if (detector1.getLocation()== Detector.Location.LEFT && turretMotor.getCurrentPosition() > -390) {
            setState(State.LEFT);
        } else if (detector1.getLocation()== Detector.Location.RIGHT && turretMotor.getCurrentPosition() < 390) {
            setState(State.RIGHT);
        }else{
            setState(State.IDLE);
        }
    }

}
