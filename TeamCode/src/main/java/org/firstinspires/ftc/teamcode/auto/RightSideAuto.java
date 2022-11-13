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
    Pose2d startPose = new Pose2d(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        Trajectory preload1 = robot.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(24, -2)).build();

        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .strafeRight(36)

                .build();
        Trajectory preload3 = robot.trajectoryBuilder(preload2.end())
                .forward(2)

                .build();
        waitForStart();
        if(isStopRequested()) return;
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        robot.followTrajectory(preload2);
        robot.followTrajectory(preload3);
    }
}
