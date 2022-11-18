package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Turret
{
    static final double     COUNTS_PER_MOTOR_REV    = 65;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 8.26771654;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TURN_SPEED             = 1;
    static final int LEFT_POS = 367, RIGHT_POS = -367, ZERO_POS = 0;
    double endPosition;
    public DcMotorEx turretMotor;
    public TouchSensor magnetic;
    public Turret.State state;
    public int prevPositionReset = 0, position = 0;

    public enum State
    {
        IDLE, MOVING, LEFT, RIGHT, ZERO
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
            case RIGHT:
                turretMotor.setTargetPosition(RIGHT_POS);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.4);
                break;
            case LEFT:
                turretMotor.setTargetPosition(LEFT_POS);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.4);
                break;
            case ZERO:
                turretMotor.setTargetPosition(ZERO_POS);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(.4);
                break;
            case MOVING:

            case IDLE:

        }
    }
    public void zero(){
        turretMotor.setTargetPosition(0);
        turretMotor.setTargetPositionTolerance(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(getState()==State.LEFT) {
            turretMotor.setPower(1);
        }else{
            turretMotor.setPower(-1);
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
