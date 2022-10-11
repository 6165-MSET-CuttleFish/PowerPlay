
package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret
{

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
        return this.state;
    }


    public void setState(Turret.State state)
    {
        this.state = state;
        update();
    }


}
