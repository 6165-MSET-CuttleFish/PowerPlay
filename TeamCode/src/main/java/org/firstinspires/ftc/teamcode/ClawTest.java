package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ClawTest extends LinearOpMode
{
    Claw claw;
    ElapsedTime time;
    @Override
    public void runOpMode() throws InterruptedException
    {

        time=new ElapsedTime();
        claw=new Claw(hardwareMap);

        waitForStart();

        claw.update();

        time.reset();
        while(time.seconds()<3)
        {

        }

        claw.setState(Claw.State.PARTIAL);
        claw.update();

        time.reset();
        while(time.seconds()<3)
        {

        }

        claw.setState(Claw.State.CLOSE);
        claw.update();

        time.reset();
        while(time.seconds()<3)
        {

        }
    }
}
