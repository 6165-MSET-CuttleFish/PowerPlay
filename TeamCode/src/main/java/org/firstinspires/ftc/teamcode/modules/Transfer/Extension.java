package org.firstinspires.ftc.teamcode.modules.Transfer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModule;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.AdvancedModuleState;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModuleState;


public class Extension extends BasicModule {
    public static double LEXTENDED = 0.28;
    public static double REXTENDED = 0.22;
    public static double LZERO = 0.06;
    public static double RZERO = 0.01;

    Servo leftExtension, rightExtension;

    public enum State implements BasicModuleState
    {
        RETRACTED(LZERO, RZERO), EXTENDED(LEXTENDED, REXTENDED);
        private final double leftPosition;
        private final double rightPosition;
        State(double leftPosition, double rightPosition)
        {
            this.leftPosition=leftPosition;
            this.rightPosition=rightPosition;
        }

        @Override
        public Double[] getValue() {
            return new Double[]{leftPosition, rightPosition};
        }

    }

    public Extension(HardwareMap hardwareMap)
    {
        super();
        leftExtension=hardwareMap.get(Servo.class, "lExt");
        rightExtension=hardwareMap.get(Servo.class, "rExt");
        rightExtension.setDirection(Servo.Direction.REVERSE);
        setState(State.RETRACTED);
    }

    @Override
    public void update()
    {
        leftExtension.setPosition(state.getValue()[0]);
        rightExtension.setPosition(state.getValue()[1]);
    }
}
