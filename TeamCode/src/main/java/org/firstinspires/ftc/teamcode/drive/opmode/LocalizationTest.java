package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotTemp;

import java.util.List;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    double prevHeading;
    double biggestDelta;

    @Override
    public void runOpMode() throws InterruptedException {

        prevHeading=0;

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotTemp drive = new RobotTemp(this);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            if(Math.abs(prevHeading-poseEstimate.getHeading())>Math.abs(biggestDelta)&&Math.abs(prevHeading- poseEstimate.getHeading())<280)
            {
                biggestDelta=Math.abs(prevHeading-poseEstimate.getHeading());
            }
            List<Double> wheelPos = drive.getWheelPositions();
            telemetry.addData("Left Encoder", wheelPos.get(0));
            telemetry.addData("Right Encoder", wheelPos.get(1));
            telemetry.addData("Back Encoder", wheelPos.get(2));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("biggest theta", biggestDelta);
            telemetry.update();

            prevHeading=Math.toDegrees(poseEstimate.getHeading());
        }
    }
}
