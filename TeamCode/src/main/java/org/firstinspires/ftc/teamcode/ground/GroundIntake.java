package org.firstinspires.ftc.teamcode.ground;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.moduleUtil.BasicModule;

public class GroundIntake extends BasicModule
{
    //temporary values
    DcMotor intakeRunning;
    public enum State implements ModuleState
    {
        INTAKING(0.7), EXTAKING(-0.7), OFF(0);
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

    public GroundIntake(HardwareMap hardwareMap)
    {
        super();
        intakeRunning=hardwareMap.get(DcMotor.class, "ground");
//        distSens = hardwareMap.get(DistanceSensor.class, "distanceG");
        setState(State.OFF);
    }

    public void update()
    {
        intakeRunning.setPower(state.getValue());
    }
}
