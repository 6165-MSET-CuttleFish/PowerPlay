package org.firstinspires.ftc.teamcode.Modules.Transfer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.BasicModule;

public class Intake extends BasicModule
{
    //temporary values

    CRServo intakeRunning;
    CRServo intakeSupporting;
    public enum State implements ModuleState
    {
        INTAKING(1), DEPOSITING(-1), OFF(0);
        private final double power;
        State(double power)
        {
            this.power=power;
        }
        @Override
        public Double getValue() {
            return power;
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
       intakeRunning.setPower(state.getValue());
       intakeSupporting.setPower(state.getValue());
    }
}
