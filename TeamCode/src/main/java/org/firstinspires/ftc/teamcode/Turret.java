package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret
{

    DcMotor turretMotor;
    public Turret.State state;
    public enum State
    {
        IDLE, LEFT_ROTATE, RIGHT_ROTATE
    }

    public Turret(HardwareMap hardwareMap)
    {
        turretMotor = hardwareMap.get(DcMotor.class, "hturret");
        setState(State.IDLE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update()
    {
        switch(state)
        {
            case IDLE:
                turretMotor.setPower(0);

            case LEFT_ROTATE:
                turretMotor.setPower(-0.25);

            case RIGHT_ROTATE:
                turretMotor.setPower(0.25);

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
