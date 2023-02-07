package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class colorSensorTest extends LinearOpMode
{
    ColorRangeSensor test;

    @Override
    public void runOpMode() throws InterruptedException
    {
        test=hardwareMap.get(ColorRangeSensor.class, "giDist");
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("distance", test.getDistance(DistanceUnit.INCH));
            telemetry.addData("blue", test.blue());
            telemetry.addData("red", test.red());
            telemetry.update();
        }
    }
}
