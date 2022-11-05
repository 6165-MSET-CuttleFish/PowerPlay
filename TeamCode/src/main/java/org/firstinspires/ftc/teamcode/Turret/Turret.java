
package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret
{

    public DcMotor turretMotor;
    public Turret.State state;
    public enum State
    {
        IDLE, MOVING
    }

    public Turret(HardwareMap hardwareMap)
    {
        turretMotor = hardwareMap.get(DcMotor.class, "hturret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void zero(){
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
    }

    public Turret.State getState() {
        return this.state;
    }


    public void setState(Turret.State state)
    {
        this.state = state;
        update();
    }


}
