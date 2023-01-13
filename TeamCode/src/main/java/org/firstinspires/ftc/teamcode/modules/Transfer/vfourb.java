package org.firstinspires.ftc.teamcode.modules.Transfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.moduleUtil.AdvancedModuleState;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModule;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModuleState;

@Config
public class vfourb extends BasicModule
{
    //temporary values
    public static double INTAKE_POSITION = 1;

    public static double DEPOSIT_POSITION = 0.17;
    public static double PRIMED_POS = 0.86;
    public static double STACK_LOW = 0.63;
    public static double ALIGN_POSITION = 0.32;
    public static double STACK_PRIMED = 0.79;

    public static double VERTICAL = 0.5;
    public static double INIT=0.86;
    Servo Running;
    Servo Supporting;

    public enum State implements BasicModuleState {
        INTAKE_POSITION(vfourb.INTAKE_POSITION),DEPOSIT_POSITION(vfourb.DEPOSIT_POSITION),
        PRIMED(PRIMED_POS), ALIGN_POSITION(vfourb.ALIGN_POSITION),
        STACK_PRIMED(vfourb.STACK_PRIMED), VERTICAL(vfourb.VERTICAL), STACK_LOW(vfourb.STACK_LOW),
        INIT(vfourb.INIT);
        private final double position;
        State(double position)
        {
            this.position=position;
        }
        @Override
        public Double[] getValue() {
            return new Double[]{position};
        }
    }

    public vfourb(HardwareMap hardwareMap)
    {
        super();
        Running = hardwareMap.get(Servo.class, "v4bRun");
        Supporting = hardwareMap.get(Servo.class, "v4bSup");
        setState(State.INIT);
    }

    public void update()
    {
        Running.setPosition(state.getValue()[0]);
        Supporting.setPosition(1-state.getValue()[0]);
    }
}