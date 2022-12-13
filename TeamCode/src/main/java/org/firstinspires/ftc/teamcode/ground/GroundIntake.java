package org.firstinspires.ftc.teamcode.ground;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.HardwareModule;
import org.firstinspires.ftc.teamcode.util.ModuleState;

public class GroundIntake extends HardwareModule
{
    //temporary values
    CRServo intakeRunning;
    CRServo intakeSupporting;
    public State state;
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
        @Override
        public Integer specialCode() {
            return null;
        }
    }

    public GroundIntake(HardwareMap hardwareMap)
    {
        intakeRunning=hardwareMap.get(CRServo.class, "gr");
        intakeSupporting = hardwareMap.get(CRServo.class, "gl");
//        distSens = hardwareMap.get(DistanceSensor.class, "distanceG");
        intakeSupporting.setDirection(CRServo.Direction.REVERSE);
        setState(State.OFF);
    }

    public void update()
    {
        intakeRunning.setPower(state.getValue());
        intakeSupporting.setPower(state.getValue());
    }
}
