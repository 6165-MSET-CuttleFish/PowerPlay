package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
//import org.firstinspires.ftc.teamcode.util.Async;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class LeftSideAutoMatew extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    ElapsedTime t;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(36, 62, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        t=new ElapsedTime();
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
                .addDisplacementMarker(2, () -> {
                    prepDepositInitial();
                })
                .lineToSplineHeading(new Pose2d(34, 23, Math.toRadians(270)))
                .addDisplacementMarker(()->
                        deposit()
                )
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
                    waitSec(0.5);
                    groundIntake.setState(GroundIntake.State.OFF);
                })
                .build();

        Trajectory toIntake = robot.trajectoryBuilder(new Pose2d(34,12,Math.toRadians(0)))
                .addDisplacementMarker(0, () -> {
                    prepIntake();
                })
                .lineToConstantHeading(new Vector2d(60  , 12))
                .addDisplacementMarker(() -> {
                    intake();
                })
                .build();

        Trajectory scoreMidCycle = robot.trajectoryBuilder(toIntake.end())
                .addTemporalMarker(0, ()->
                {
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .addTemporalMarker(0.5, ()->
                {
                    slides.setState(Slides.State.BOTTOM);
                })
                .addTemporalMarker(1.25, ()->
                {
                    prepDeposit();
                })
                .lineToConstantHeading(new Vector2d(23.5, 14))
                .addDisplacementMarker(()->
                        deposit()
                )
                .build();
        Trajectory toIntakePostCycle = robot.trajectoryBuilder(new Pose2d(23.5,14,Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    prepIntake();
                })
                .lineToConstantHeading(new Vector2d(60,12), Robot.getVelocityConstraint(35,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), Robot.getAccelerationConstraint(15))
                .addDisplacementMarker(()->{
                    intake();
                })
                .build();




        waitForStart();
        if (isStopRequested()) return;

        robot.setPoseEstimate(startPose);
        robot.followTrajectory(scorePreload);
        robot.followTrajectory(removeSignal);
        robot.turn(Math.toRadians(90));

        robot.followTrajectory(toIntake);
        robot.followTrajectory(scoreMidCycle);
        robot.followTrajectory(toIntakePostCycle);
        robot.followTrajectory(scoreMidCycle);
        robot.followTrajectory(toIntakePostCycle);
        robot.followTrajectory(scoreMidCycle);
    }

    public void prepIntake()
    {
        slides.setState(Slides.State.INTAKE_AUTO);
        fourbar.setState(vfourb.State.PRIMED);
        turret.setState(Turret.State.ZERO);
    }
    public void prepDeposit()
    {
        slides.setState(Slides.State.MID_DROP);
        fourbar.setState(vfourb.State.ALIGN_POSITION);
        turret.setState(Turret.State.RIGHT);
    }
    public void prepDepositInitial()
    {
        groundIntake.setState(GroundIntake.State.INTAKING);
        turret.setState(Turret.State.LEFT);
        slides.setState(Slides.State.MID_DROP);
        fourbar.setState(vfourb.State.ALIGN_POSITION);
    }
    public void intake()
    {
        fourbar.setState(vfourb.State.INTAKE_POSITION);
        intake.setState(Intake.State.INTAKING);
        waitSec(1);
    }
    public void deposit()
    {
        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        waitSec(1);
        this.intake.setState(Intake.State.DEPOSITING);
        waitSec(0.5);
    }
    public void waitSec(double seconds)
    {
        t.reset();
        while(t.milliseconds()<seconds*1000)
        {
            //stall
        }
    }
}
