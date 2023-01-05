package org.firstinspires.ftc.teamcode.modules.transfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class v4bTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        Robot r=new Robot(this, false);
        vfourb v4b=r.fourbar;

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                v4b.setState(vfourb.State.STACK_PRIMED);
            }
            else if(gamepad1.b)
            {
                v4b.setState(vfourb.State.PRIMED);

            }
            else if(gamepad1.y)
            {
                v4b.setState(vfourb.State.STACK_LOW);
            }
            else if(gamepad1.x)
            {
                v4b.setState(vfourb.State.VERTICAL);
            }
            else if(gamepad1.dpad_down)
            {
                v4b.setState(vfourb.State.INTAKE_POSITION);
            }
        }
    }
}
