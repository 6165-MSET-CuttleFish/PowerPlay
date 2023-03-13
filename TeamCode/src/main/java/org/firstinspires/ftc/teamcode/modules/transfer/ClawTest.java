package org.firstinspires.ftc.teamcode.modules.transfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp
public class ClawTest extends LinearOpMode
{
    Claw claw;
    ElapsedTime time;
    @Override
    public void runOpMode() throws InterruptedException
    {
        Context.hardwareMap=hardwareMap;
        time=new ElapsedTime();
        claw=new Claw();
        claw.setState(Claw.State.OPEN);
        waitForStart();
        while(opModeIsActive()){
            claw.update();
            if(gamepad1.a) claw.setState(Claw.State.OPEN);
            else if (gamepad1.b) claw.setState(Claw.State.CLOSE);
        }
    }
}
