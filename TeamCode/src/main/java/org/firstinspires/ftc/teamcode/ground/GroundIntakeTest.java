package org.firstinspires.ftc.teamcode.ground;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.moduleUtil.BasicModule;

@TeleOp
public class GroundIntakeTest extends LinearOpMode
{
    GroundIntake groundIntake;

    @Override
    public void runOpMode() throws InterruptedException
    {
        groundIntake=new GroundIntake(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                groundIntake.setState(GroundIntake.State.INTAKING);
            }
            else if(gamepad1.b)
            {
                groundIntake.setState(GroundIntake.State.EXTAKING);
            }
            else
            {
                groundIntake.setState(GroundIntake.State.OFF);
            }
        }

    }
}
