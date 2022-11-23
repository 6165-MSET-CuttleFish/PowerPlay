package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class LeftSideAuto extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose =new Pose2d(36, 62, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourbar.setState(vfourb.State.INTAKE_POSITION);

        Trajectory scorePreload = robot.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(34,23,Math.toRadians(270)))
                .addDisplacementMarker(2, () -> {
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    turret.setState(Turret.State.LEFT);
                    slides.setState(Slides.State.MID_DROP);
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .addTemporalMarker(2, () -> {
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                })
                .addTemporalMarker(2.1, () -> {
                    intake.setState(Intake.State.DEPOSITING);
                })
                .build();

        Trajectory removeSignal = robot.trajectoryBuilder(scorePreload.end())
                .addTemporalMarker(0, () -> {
                    fourbar.setState(vfourb.State.PRIMED);
                    slides.setState(Slides.State.BOTTOM);
                    turret.setState(Turret.State.ZERO);
                })
                .lineToConstantHeading(new Vector2d(34,12))
                .addTemporalMarker(1,()->{
                    groundIntake.setState(GroundIntake.State.DEPOSITING);
                    intake.setState(Intake.State.OFF);
                })
                .build();

        Trajectory toIntake = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(59,12))
                .addDisplacementMarker(0, () -> {
                    intake.setState(Intake.State.INTAKING);
                })
                .build();

        Trajectory scoreMidCycle = robot.trajectoryBuilder(toIntake.end())
                .addDisplacementMarker(0, () -> {
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    turret.setState(Turret.State.LEFT);
                    slides.setState(Slides.State.MID_DROP);
                })
                .lineToConstantHeading(new Vector2d(23.5,14))
                .addTemporalMarker(2, () -> {
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                })
                .addTemporalMarker(2.1, () -> {
                    intake.setState(Intake.State.DEPOSITING);
                })
                .build();

        /*Trajectory cycleIntake = robot.trajectoryBuilder(preload3.end())
                        .lineToConstantHeading(new Vector2d(-48,10))
                                .build();*/

        waitForStart();
        if (isStopRequested()) return;
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(scorePreload);
        robot.followTrajectory(removeSignal);
        wait(50);
        robot.groundIntake.setState(GroundIntake.State.OFF);
        robot.turn(Math.toRadians(90));
        robot.followTrajectory(toIntake);
        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);
        wait(50);
        robot.fourbar.setState(vfourb.State.ALIGN_POSITION);
        wait(50);
        robot.followTrajectory(scoreMidCycle);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
