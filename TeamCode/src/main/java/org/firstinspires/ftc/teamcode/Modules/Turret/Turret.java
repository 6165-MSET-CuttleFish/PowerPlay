package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Detection.pipelines.Detector;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.AdvancedModuleWorker;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.AdvancedModule;
import org.firstinspires.ftc.teamcode.Modules.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.util.PIDCoeff;
import org.firstinspires.ftc.teamcode.util.PIDControl;

@Config
public class Turret extends AdvancedModule
{
    PIDCoeff coeff=new PIDCoeff(0.1, 0, 0, 0, 0);
    PIDControl controller=new PIDControl(coeff);

    static final double LEFT_POS = -2100, RIGHT_POS = 2100, ZERO_POS = 0, INIT_POS=1020;

    public DcMotorEx turretMotor;
    public Encoder encoder;
    public TouchSensor limit;

    double alignPos;
    Detector detector;

    public enum State implements ModuleState
    {
        LEFT(LEFT_POS), RIGHT(RIGHT_POS), ZERO(ZERO_POS),
        MANUAL(null), AUTOALIGN(null), STOPPED(null), INIT(INIT_POS);

        private final Double position;

        State(Double position)
        {
            this.position=position;
        }
        @Override
        public Double getValue()
        {
            return position;
        }
    }

    public Turret(HardwareMap hardwareMap, boolean teleop)
    {
        turretMotor=hardwareMap.get(DcMotorEx.class, "hturret");
        motors.add(turretMotor);
        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        limit = hardwareMap.get(TouchSensor.class, "limit");
        timer=new ElapsedTime();
        detector=new Detector();

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //do the init state thing based on teleop
        setState(State.ZERO);
        w=new AdvancedModuleWorker(this);
        w.start();
    }

    public void autoAlign()
    {
        setState(State.AUTOALIGN);
        targetPos=currentPos()+detector.getDistance();
    }

    public double motorPower()
    {
        if(state==State.MANUAL)
            return manualPower;
        else if(state==State.STOPPED)
            return 0;

        double power=controller.calculate(currentPos(), targetPos, timer.milliseconds());
        if(Math.abs(power)<0.08)
            setState(State.STOPPED);
        return power;
    }
    public boolean resetPressed() {return limit.isPressed();}
    public double currentPos() {return encoder.getCurrentPosition();}
}