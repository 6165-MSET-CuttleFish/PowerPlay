package org.firstinspires.ftc.teamcode.Modules.Transfer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.moduleUtil.BasicModule;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.ModuleState;


public class Extension extends BasicModule {
    Servo servo;
    public static double NEUTRAL_POSITION=0;

    public enum State implements ModuleState
    {
        NEUTRAL(NEUTRAL_POSITION);
        private final double position;
        State(double position){this.position=position;}

        @Override
        public Double getValue() {
            return position;
        }
    }

    public Extension(HardwareMap hardwareMap)
    {
        super();
        servo=hardwareMap.get(Servo.class, "extension");
        setState(State.NEUTRAL);
    }

    @Override
    public void update()
    {
        servo.setPosition(state.getValue());
    }
}
