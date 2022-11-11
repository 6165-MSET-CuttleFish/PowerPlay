package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
@Autonomous
public class quickpark extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        Trajectory trajectory = robot.trajectoryBuilder(new Pose2d())
                .strafeLeft(28)
                .build();
        Trajectory trajectory1 = robot.trajectoryBuilder(new Pose2d())
                .back(30)
                .build();
        waitForStart();
        if (isStopRequested()) return;

        robot.followTrajectory(trajectory);
        robot.followTrajectory(trajectory1);
        Pose2d poseEstimate = robot.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
