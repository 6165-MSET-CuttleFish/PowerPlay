package org.firstinspires.ftc.teamcode.modules.Transfer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.moduleUtil.AdvancedModuleState;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModule;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModuleState;

public class Intake extends BasicModule
{
    //temporary values

    CRServo intakeRunning;
    CRServo intakeSupporting;
    public enum State implements BasicModuleState
    {
        INTAKING(1), DEPOSITING(-1), OFF(0);
        private final double power;
        State(double power)
        {
            this.power=power;
        }
        @Override
        public Double[] getValue() {
            return new Double[]{power};
        }
    }

    public Intake(HardwareMap hardwareMap)
    {
        super();
        intakeRunning = hardwareMap.get(CRServo.class, "intakeRun");
        intakeSupporting = hardwareMap.get(CRServo.class, "intakeSup");
        intakeSupporting.setDirection(CRServo.Direction.REVERSE);
        setState(State.OFF);
    }

    public void update()
    {
       intakeRunning.setPower(state.getValue()[0]);
       intakeSupporting.setPower(state.getValue()[0]);
    }
}
