package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.RobotTemp;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;

@TeleOp
public class SchedulerTestExample extends LinearOpMode
{
    RobotTemp r;
    Turret turret;
    Slides slides;
    TaskScheduler scheduler;



    @Override
    public void runOpMode() throws InterruptedException
    {
        r=new RobotTemp(this);
        slides=r.slides;
        turret=r.turret;
        scheduler=new TaskScheduler(this);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                //run slides 500 miliseconds after left bumper has been pressed
                scheduler.scheduleTask(slides, Slides.State.HIGH, 500, ()->gamepad2.left_bumper==true);
            }
            else if(gamepad1.y)
            {
                //run slides after dpad down has been pressed(you can swap this with an action like turret position completed)
                scheduler.scheduleTask(turret, Turret.State.BACK);
            }
            else if(gamepad1.x)
            {
                //run slides 2 seconds after x pressed
                scheduler.scheduleTask(turret, Turret.State.LEFT);
            }
            else if(gamepad1.b)
            {
                scheduler.scheduleTask(turret, Turret.State.RIGHT);
            }
            telemetry.addData("currentPos: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("currentVelo: ", slides.slidesLeft.getVelocity());
            telemetry.addData("currentPower: ", slides.slidesLeft.getPower());
            telemetry.addData("currentPos: ", slides.slidesRight.getCurrentPosition());
            telemetry.addData("currentVelo: ", slides.slidesRight.getVelocity());
            telemetry.addData("currentPower: ", slides.slidesRight.getPower());
            telemetry.addData("Limit Switch: ", slides.slidesLimitSwitch.getVoltage());
            telemetry.addData("State: ", slides.getState());
            telemetry.update();
        }
    }
}
