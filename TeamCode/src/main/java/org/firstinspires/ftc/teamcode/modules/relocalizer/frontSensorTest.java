package org.firstinspires.ftc.teamcode.modules.relocalizer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.relocalizer.filter;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class frontSensorTest extends LinearOpMode {

    AnalogInput distanceInput;
    MB1242 test;
    filter movingMedianFront;
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        test = robot.distfl;
        movingMedianFront = new filter(10);

        waitForStart();
        while(opModeIsActive()) {
            robot.deposit.setExtension(Deposit.ExtensionState.EXTEND);
            double rawFront = test.getDistance(DistanceUnit.INCH);

            double filterFront = movingMedianFront.update(rawFront);

            telemetry.addData("RAW Front Distance", rawFront);

            telemetry.addData("FILTER Front Distance", filterFront);

            telemetry.update();
        }
    }
/*
    public double getDistance(){
        return distanceInput.getVoltage();
    }*/
}
