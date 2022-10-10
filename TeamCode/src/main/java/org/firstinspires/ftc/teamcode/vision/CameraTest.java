package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.Camera;

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
