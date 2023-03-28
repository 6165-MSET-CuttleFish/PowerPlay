package org.firstinspires.ftc.teamcode.modules.relocalizer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.relocalizer.filter;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class frontSensorTest extends LinearOpMode {

    AnalogInput distanceInput;
    MB1242 test;

    public MB1242 distfl, distfr;

    filter movingMedianFront;
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        //test = robot.distfl;
        movingMedianFront = new filter(10);

        distfl = hardwareMap.get(MB1242.class, "frontLeftDistance");
        distfr = hardwareMap.get(MB1242.class, "frontRightDistance");

        waitForStart();
        while(opModeIsActive()) {
            robot.deposit.setExtension(Deposit.ExtensionState.EXTEND);

            // double rawFront = test.getDistance(DistanceUnit.INCH);
            //double rawSide = robot.right.getDistance(DistanceUnit.INCH);
            //double filterFront = movingMedianFront.update(rawFront);
            //double filterSide = movingMedianFront.update(rawSide);
            //robot.localizer.update();
            //telemetry.addData("front left", robot.localizer.frontDistL);
            //telemetry.addData("front right", robot.localizer.frontDistR);
            //telemetry.addData("left", robot.localizer.leftDist);
            //telemetry.addData("right", robot.localizer.rightDist);
            telemetry.addData("Left", distfl.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right", distfr.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
/*
    public double getDistance(){
        return distanceInput.getVoltage();
    }*/
}
