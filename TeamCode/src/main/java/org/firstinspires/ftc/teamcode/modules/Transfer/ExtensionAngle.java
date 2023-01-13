package org.firstinspires.ftc.teamcode.modules.Transfer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModule;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.AdvancedModuleState;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModuleState;

public class ExtensionAngle extends BasicModule {
    Servo leftAngular, rightAngular;
    public static double LVECTORING = 0.22;
    public static double RVECTORING = 0.23;
    public static double LINTAKE = 0.37;
    public static double RINTAKE = 0.36;

    public enum State implements BasicModuleState
    {
        VECTORING(LVECTORING, RVECTORING), INTAKE(LINTAKE, RINTAKE);
        private final double leftPosition, rightPosition;
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

    public ExtensionAngle(HardwareMap hardwareMap)
    {
        super();
        leftAngular = hardwareMap.get(Servo.class, "lAng");
        rightAngular= hardwareMap.get(Servo.class, "rAng");
        rightAngular.setDirection(Servo.Direction.REVERSE);
        setState(State.INTAKE);
    }

    @Override
    public void update()
    {
        leftAngular.setPosition(state.getValue()[0]);
        rightAngular.setPosition(state.getValue()[1]);
    }
}
