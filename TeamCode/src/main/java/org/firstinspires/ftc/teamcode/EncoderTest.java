package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;
@TeleOp
public class EncoderTest extends LinearOpMode
{
    Encoder leftEncoder;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftEncoder=new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("encoder", leftEncoder.getCurrentPosition());
            telemetry.update();
        }

    }
}
