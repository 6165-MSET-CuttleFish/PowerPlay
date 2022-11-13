package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
@Autonomous
public class RightSideAuto extends LinearOpMode {
    Robot robot;
    Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
    Trajectory preload = robot.trajectoryBuilder(new Pose2d())
            .lineToConstantHeading(new Vector2d(2, 24))
            .lineToConstantHeading(new Vector2d(30, 24))
            .lineToConstantHeading(new Vector2d(30,26))
            .build();
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        if(isStopRequested()) return;
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload);
    }
}
