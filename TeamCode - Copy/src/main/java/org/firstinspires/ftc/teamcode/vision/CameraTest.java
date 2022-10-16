package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.Camera;

public class CameraTest extends LinearOpMode
{
    FtcDashboard dashboard;
    TelemetryPacket packet;
    Camera camera;

    @Override
    public void runOpMode() throws InterruptedException
    {
        packet=new TelemetryPacket();
        dashboard=FtcDashboard.getInstance();

        camera=new Camera(hardwareMap, telemetry);
        dashboard.startCameraStream(camera.streamSource(),30);

        waitForStart();
        while(opModeIsActive())
        {
            packet.put("Zone:", camera.getState());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Zone:", camera.getState());
            telemetry.update();
        }
    }
}
