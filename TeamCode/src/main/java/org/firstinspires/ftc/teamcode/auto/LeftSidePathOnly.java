package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.Slides.Slides;
import org.firstinspires.ftc.teamcode.modules.Transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.detection.visionProcessing.Detector;
import org.firstinspires.ftc.teamcode.modules.Turret.Turret;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.openftc.easyopencv.OpenCvWebcam;

//@Autonomous
public class LeftSidePathOnly extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(36, 62, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourbar.setState(vfourb.State.INTAKE_POSITION);

        Trajectory scorePreload = robot.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(34, 23, Math.toRadians(270)))
                .build();

        Trajectory removeSignal = robot.trajectoryBuilder(scorePreload.end())
                .lineToConstantHeading(new Vector2d(34, 12))
                .build();

        Trajectory toIntake = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(59, 12))
                .build();

        Trajectory scoreMidCycle = robot.trajectoryBuilder(toIntake.end())
                .lineToConstantHeading(new Vector2d(23.5, 13))
                .build();
        
        waitForStart();
        if (isStopRequested()) return;
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(scorePreload);
        robot.followTrajectory(removeSignal);
        sleep(200);
        robot.turn(Math.toRadians(90));
        robot.followTrajectory(toIntake);
        sleep(200);
        sleep(200);
        robot.followTrajectory(scoreMidCycle);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
