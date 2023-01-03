package org.firstinspires.ftc.teamcode.modules.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.transfer.vfourb;

@TeleOp
public class Turret2Test extends LinearOpMode
{
    Turret turret;
    Robot robot;
    vfourb fourbar;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot=new Robot(this, false);
        turret=robot.turret;
        fourbar = robot.fourbar;
        timer=new ElapsedTime();
        fourbar.setState(vfourb.State.PRIMED);

        while(!isStarted())
        {
            telemetry.addData("encoder", turret.encoder.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();




        //turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //turret.setState(Turret2.State.LEFT);
        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                turret.setState(Turret.State.RIGHT);
            }
            else if(gamepad1.b)
            {
                turret.setState(Turret.State.LEFT);
            }
            else if(gamepad1.y)
            {
                turret.setState(Turret.State.ZERO);
            }
            //turret.update();
            telemetry.addData("State", turret.getState());
            telemetry.addData("power", turret.motorOil);
            telemetry.addData("encoder", turret.encoder.getCurrentPosition());
            //telemetry.addData("pid", turret.pidMotorOil);
            telemetry.update();
        }
    }
}
