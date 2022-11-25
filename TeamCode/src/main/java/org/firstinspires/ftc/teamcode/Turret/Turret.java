package org.firstinspires.ftc.teamcode.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.easyopencv.OpenCvWebcam;
@Config
public class Turret
{
    static final double     COUNTS_PER_MOTOR_REV    = 65;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 8.26771654;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TURN_SPEED             = 1;
    static final int LEFT_POS = -380, RIGHT_POS = 380, ZERO_POS = 0;//through bore L = -2049, R = 1990, 0 = 20
    double endPosition;
    public DcMotorEx turretMotor;
    public TouchSensor magnetic;
    public Turret.State state;
    public int prevPositionReset = 0, position = 0;
    public static PIDFCoefficients TURRET_PIDF = new PIDFCoefficients(1.502, 0, 0, 0);
    public PIDController pidController;
    public enum State {
        IDLE, MOVING, LEFT, RIGHT, ZERO, MANUAL
    }

    public Turret(HardwareMap hardwareMap)
    {
        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");
        magnetic = hardwareMap.get(TouchSensor.class, "MLS");
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //turretMotor.setDirection(DcMotor.Direction.REVERSE);
        setState(State.IDLE);
    }

    public void update()
    {
        turretMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, TURRET_PIDF);
        switch(state)
        {
            case MOVING:
                break;
            case IDLE:
                turretMotor.setPower(0);
                break;
            case RIGHT:
                turretMotor.setTargetPosition(RIGHT_POS);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.5);
                break;
            case LEFT:
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

}
