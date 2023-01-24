package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CoroutineTester extends LinearOpMode
{
    private FtcDashboard dashboard=FtcDashboard.getInstance();
    TelemetryPacket packet=new TelemetryPacket();


    TestCoroutine cr;
    @Override
    public void runOpMode() throws InterruptedException
    {
        cr=new TestCoroutine(telemetry, dashboard, packet);
        cr.startCoroutine();
        waitForStart();
        while(opModeIsActive())
        {
            //stall
        }
    }
}
