package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake
{

    CRServo intake1, intake2;
    public Intake.State state;
    public enum State
    {
        IN, OUT, OFF
    }

    public Intake(HardwareMap hardwareMap)
    {
        intake1=hardwareMap.get(CRServo.class, "intake1");
        intake2=hardwareMap.get(CRServo.class, "intake2");

        setState(State.OFF);
    }

    public void update()
    {
        switch(state)
        {
            case IN:
                intake1.setPower(1);
                intake2.setPower(1);
            case OUT:
                intake1.setPower(-1);
                intake2.setPower(-1);
            case OFF:
                intake1.setPower(0);
                intake2.setPower(0);
        }
    }

    public Intake.State getState() {
        return state;
    }

    public void setState(Intake.State state)
    {
        this.state = state;
        update();
    }

}
