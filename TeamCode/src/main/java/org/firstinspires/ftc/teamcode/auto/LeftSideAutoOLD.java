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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class LeftSideAutoOLD extends LinearOpMode {
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
        robot = new Robot(this, false);
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
                .lineToConstantHeading(new Vector2d(34, 12))
                .addTemporalMarker(1, () -> {
                    groundIntake.setState(GroundIntake.State.DEPOSITING);
                    intake.setState(Intake.State.OFF);
                })
                .build();

        Trajectory toIntake = robot.trajectoryBuilder(new Pose2d(34,12,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(61  , 12))
                .addDisplacementMarker(0, () -> {
                    intake.setState(Intake.State.INTAKING);
                    slides.setState(Slides.State.LOW_DROP);
                })
                .build();

        Trajectory scoreMidCycle = robot.trajectoryBuilder(toIntake.end())
                .addDisplacementMarker(0, () -> {
                    turret.setState(Turret.State.RIGHT);
                    slides.setState(Slides.State.MID_DROP);
                })
                .lineToConstantHeading(new Vector2d(23.5, 14))
                .addTemporalMarker(2, () -> {
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                })
                .addTemporalMarker(2.1, () -> {
                    intake.setState(Intake.State.DEPOSITING);
                })
                .build();
        Trajectory toIntakePostCycle = robot.trajectoryBuilder(new Pose2d(23.5,14,Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    fourbar.setState(vfourb.State.PRIMED);
                    slides.setState(Slides.State.BOTTOM);
                    turret.setState(Turret.State.ZERO);
                })
                .lineToConstantHeading(new Vector2d(61,12), Robot.getVelocityConstraint(35,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), Robot.getAccelerationConstraint(15))
                .addTemporalMarker(1, () -> {
                    intake.setState(Intake.State.INTAKING);
                    slides.setState(Slides.State.LOW_DROP);
                })
                .build();




        waitForStart();
        if (isStopRequested()) return;
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(scorePreload);
        robot.followTrajectory(removeSignal);
        sleep(500);
        robot.groundIntake.setState(GroundIntake.State.OFF);
        robot.turn(Math.toRadians(90));

        robot.followTrajectory(toIntake);
        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);
        sleep(1000);
        robot.fourbar.setState(vfourb.State.ALIGN_POSITION);
        sleep(1000);
        robot.followTrajectory(scoreMidCycle);
        sleep(500);
        robot.followTrajectory(toIntakePostCycle);

        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);
        sleep(1000);
        robot.fourbar.setState(vfourb.State.ALIGN_POSITION);
        sleep(1000);
        robot.followTrajectory(scoreMidCycle);
        sleep(500);
        robot.followTrajectory(toIntakePostCycle);

        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);
        sleep(1000);
        robot.fourbar.setState(vfourb.State.ALIGN_POSITION);
        sleep(1000);
        robot.followTrajectory(scoreMidCycle);
        sleep(500);
        robot.followTrajectory(toIntakePostCycle);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
