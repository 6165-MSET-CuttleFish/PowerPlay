package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class Turret2Test extends LinearOpMode
{
    Turret turret;
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot=new Robot(this);
        turret=robot.turret;
        telemetry.addData("encoder", turret.encoder.getCurrentPosition());
        waitForStart();


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
            telemetry.addData("State", turret.getState());
            telemetry.addData("power", turret.motorOil);
            telemetry.addData("encoder", turret.encoder.getCurrentPosition());
            //telemetry.addData("pid", turret.pidMotorOil);
            telemetry.update();
        }
    }
}
