package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Turret2Test extends LinearOpMode
{
    Turret2 turret;
    @Override
    public void runOpMode() throws InterruptedException
    {
        turret=new Turret2(hardwareMap);
        waitForStart();


        turret.setState(Turret2.State.LEFT);
        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                turret.setState(Turret2.State.RIGHT);
            }
            else if(gamepad1.b)
            {
                turret.setState(Turret2.State.LEFT);
            }
            else if(gamepad1.dpad_left)
            {
                turret.setState(Turret2.State.MIDRIGHT);
            }
            else if(gamepad1.dpad_right)
            {
                turret.setState(Turret2.State.MIDLEFT);
            }

            turret.update();
            telemetry.addData("State", turret.getState());
            telemetry.addData("Error", turret.motorPower());
            telemetry.update();
        }
    }
}