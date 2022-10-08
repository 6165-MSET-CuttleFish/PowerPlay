package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp

public class IntakeTest extends LinearOpMode
{
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake=new Intake(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                intake.setState(Intake.State.IN);
            }
            else if(gamepad1.b)
            {
                intake.setState(Intake.State.OUT);
            }
            else if(gamepad1.x)
            {
                intake.setState(Intake.State.OFF);
            }
        }
    }
}
