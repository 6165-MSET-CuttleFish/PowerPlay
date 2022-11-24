package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class UltrasonicTest extends LinearOpMode {
    DistanceSensor frontDist;

    @Override
    public void runOpMode() throws InterruptedException {
        frontDist=hardwareMap.get(MB1242.class, "f");

        //frontDist = new MB1242(hardwareMap,"f");

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("distance", frontDist.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
