package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret
{
    static final double     COUNTS_PER_MOTOR_REV    = 65;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 8.26771654;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TURN_SPEED             = 1;

    double endPosition;
    DcMotor turretMotor;
    public Turret.State state;
    public enum State
    {
        IDLE, MOVING
    }

    public Turret(HardwareMap hardwareMap)
    {
        turretMotor = hardwareMap.get(DcMotor.class, "hturret");
        setState(State.IDLE);
    }

    public void update()
    {
        switch(state)
        {
            case MOVING:

            case IDLE:

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
