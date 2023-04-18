package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

        //turret.factor=-1.25;
        Context.autoalignConstantSpeed=true;

        waitForStart();
        turret.setState(Turret.State.AUTOALIGN);

        while(opModeIsActive())
        {
            turret.update();

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            telemetry.addData("Target Vel", turret.autoalign.getPower());
            //telemetry.addData("High Power", turret.autoalign.controller.highPower);
            //telemetry.addData("Error", turret.autoalign.controller.error);
            telemetry.addData("State", turret.getState());
            telemetry.addData("Aligning", turret.autoalign.aligning);
            telemetry.addData("Aligning State", turret.autoalign.alignstate);
            telemetry.addData("Actual Velocity", turret.encoder.getCorrectedVelocity());
            telemetry.addData("Power", turret.turretMotor.getPower());
            //telemetry.addData("Battery", turret.getBatteryVoltage());
            telemetry.update();
        }
    }
}
