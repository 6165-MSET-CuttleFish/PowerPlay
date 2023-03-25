package org.firstinspires.ftc.teamcode.modules.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp
public class AutoalignTest extends LinearOpMode
{
    Robot robot;
    Turret turret;
    double timer=0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Context.autoalignEnabled=true;
        robot=new Robot(this);
        turret=robot.turret;

        waitForStart();

        while(opModeIsActive())
        {
            turret.setState(Turret.State.AUTOALIGN);
            turret.update();

            telemetry.addData("Shift", turret.autoalign.getShift());
            telemetry.addData("State", turret.getState());
            telemetry.update();
        }
    }
}
