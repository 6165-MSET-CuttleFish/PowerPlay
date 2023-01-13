package org.firstinspires.ftc.teamcode.modules.ground;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.moduleUtil.AdvancedModuleState;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModule;
import org.firstinspires.ftc.teamcode.modules.moduleUtil.BasicModuleState;

public class GroundIntake extends BasicModule
{
    //temporary values
    DcMotor intakeRunning;
    public enum State implements BasicModuleState
    {
        INTAKING(0.7), EXTAKING(-0.7), OFF(0);
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

    public GroundIntake(HardwareMap hardwareMap)
    {
        super();
        intakeRunning=hardwareMap.get(DcMotor.class, "ground");
//        distSens = hardwareMap.get(DistanceSensor.class, "distanceG");
        setState(State.OFF);
    }

    public void update()
    {
        intakeRunning.setPower(state.getValue()[0]);
    }
}
