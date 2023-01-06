package org.firstinspires.ftc.teamcode.Modules.Transfer;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.moduleUtil.BasicModule;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.ModuleState;

public class Claw extends BasicModule
{
    //temporary values
    static final double OPEN=0;
    static final double CLOSE=0;
    static final double PARTIAL=0;

    ServoEx claw;
    public enum State implements ModuleState
    {
        OPEN(Claw.OPEN), CLOSE(Claw.CLOSE), PARTIAL(Claw.PARTIAL);
        private final double position;
        State(double position)
        {
            this.position=position;
        }
        @Override
        public Double getValue() {
            return position;
        }
    }

    public Claw(HardwareMap hardwareMap)
    {
        super();
        claw = hardwareMap.get(ServoEx.class, "Claw");
        setState(State.OPEN);
    }

    public void update()
    {
        claw.setPosition(state.getValue());
    }
}
