package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class AlignerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot r=new Robot(this);
        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.x)
            {
                r.alignUp();
            }
            else if(gamepad1.y)
            {
                r.alignDown();
            }
        }
    }
}
