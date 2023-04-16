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
        turret.setState(Turret.State.AUTOALIGN);

        while(opModeIsActive())
        {
            turret.update();

            telemetry.addData("Power", turret.autoalign.getPower());
            //telemetry.addData("High Power", turret.autoalign.controller.highPower);
            //telemetry.addData("Error", turret.autoalign.controller.error);
            telemetry.addData("State", turret.getState());
            telemetry.addData("Aligning", turret.autoalign.aligning);
            telemetry.addData("Aligning State", turret.autoalign.alignstate);
            telemetry.update();
        }
    }
}
