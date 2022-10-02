package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CameraTest extends LinearOpMode
{
    Camera camera;

    @Override
    public void runOpMode() throws InterruptedException
    {
        camera=new Camera(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive())
        {
            camera.update();
            telemetry.addData("Status:", camera.getState());
            telemetry.update();
        }
    }
}
