package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Modules.Slides.Slides;
import org.firstinspires.ftc.teamcode.Modules.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Modules.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Modules.Turret.Detector;
import org.firstinspires.ftc.teamcode.Modules.Turret.Turret;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Modules.ground.GroundIntake;
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
        robot = new Robot(this, false);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        //turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fourbar.setState(vfourb.State.INTAKE_POSITION);

        Trajectory scorePreload = robot.trajectoryBuilder(startPose)
                .addDisplacementMarker(0.75, () -> {
                    prepDepositInitial();
                })
                .lineToConstantHeading(new Vector2d(33, 23))
                .addDisplacementMarker(()->
                        deposit()
                )
                .addDisplacementMarker(()->
                {
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    waitSec(0.2);
                })
                .build();

        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(33, 23))
                .addDisplacementMarker(1, ()->{
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .addDisplacementMarker(2, ()->{
                    groundIntake.setState(GroundIntake.State.EXTAKING);
                    turret.setState(Turret.State.LEFT);
                    //turret.update();
                    slides.setState(Slides.State.MID_DROP);

                })
                .addTemporalMarker(2,()->{
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);

                })
                .addTemporalMarker(2.1, ()->{
                    intake.setState(Intake.State.DEPOSITING);
                })
                .addDisplacementMarker(()->
                {
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    waitSec(0.25);
                })
                .build();

        Trajectory removeSignal = robot.trajectoryBuilder(scorePreload.end())
                .addTemporalMarker(0, () -> {
                    fourbar.setState(vfourb.State.PRIMED);
                    slides.setState(Slides.State.BOTTOM);
                    turret.setState(Turret.State.ZERO);
                })
                .lineToConstantHeading(new Vector2d(33, 15.5))
                .addTemporalMarker(1, () -> {
                    groundIntake.setState(GroundIntake.State.EXTAKING);
                    intake.setState(Intake.State.OFF);
                    waitSec(0.3);
                    groundIntake.setState(GroundIntake.State.OFF);
                })
                .build();

        Trajectory toIntake = robot.trajectoryBuilder(new Pose2d(34,12,Math.toRadians(0)))
                .addDisplacementMarker(0, () -> {
                    prepIntake();
                })
                .lineToLinearHeading(new Pose2d(62  , 12, Math.toRadians(0)), robot.getVelocityConstraint(37, 5.939, 13.44),
                        robot.getAccelerationConstraint(35))
                .addDisplacementMarker(() -> {
                    intake();
                })
                .addDisplacementMarker(()->
                {
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    waitSec(0.2);
                })
                .build();

        Trajectory scoreMidCycle = robot.trajectoryBuilder(toIntake.end())
                .addTemporalMarker(0, ()->
                {
                    slides.setState(Slides.State.BOTTOM);
                })
                .addTemporalMarker(0.5, ()->
                {
                    prepDeposit();
                })
                .lineToConstantHeading(new Vector2d(26.5, 14))
                .addDisplacementMarker(()->
                        deposit()
                )
                .addDisplacementMarker(()->
                        {
                            fourbar.setState(vfourb.State.ALIGN_POSITION);
                            waitSec(0.2);
                        })
                .build();
        Trajectory toIntakePostCycle = robot.trajectoryBuilder(new Pose2d(23.5,14,Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    prepIntake();
                })
                .lineToConstantHeading(new Vector2d(62,12), Robot.getVelocityConstraint(35,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), Robot.getAccelerationConstraint(15))
                .addDisplacementMarker(()->{
                    intake();
                })
                .addDisplacementMarker(()->
                {
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    waitSec(0.2);
                })
                .build();

        Trajectory toIntakePostCycleLow = robot.trajectoryBuilder(new Pose2d(23.5,14,Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    prepIntake();
                })
                .lineToConstantHeading(new Vector2d(62,12), Robot.getVelocityConstraint(35,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), Robot.getAccelerationConstraint(15))
                .addDisplacementMarker(()->{
                    intakeLow();
                })
                .addDisplacementMarker(()->
                {
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    waitSec(0.2);
                })
                .build();




        waitForStart();
        if (isStopRequested()) return;

        robot.setPoseEstimate(startPose);
        robot.followTrajectory(scorePreload);
        robot.followTrajectory(removeSignal);
        robot.turn(Math.toRadians(90));

        //robot.followTrajectory(toIntake);
        /*robot.followTrajectory(scoreMidCycle);
        robot.followTrajectory(toIntakePostCycle);
        robot.followTrajectory(scoreMidCycle);
        robot.followTrajectory(toIntakePostCycleLow);
        robot.followTrajectory(scoreMidCycle);
        robot.followTrajectory(toIntakePostCycleLow);
        robot.followTrajectory(scoreMidCycle);*/


        /*robot.setPoseEstimate(startPose);
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
        while (!isStopRequested() && opModeIsActive()) ;*/
    }

    public void prepIntake()
    {
        slides.setState(Slides.State.INTAKE_AUTO);
        fourbar.setState(vfourb.State.VERTICAL);
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
        //groundIntake.setState(GroundIntake.State.INTAKING);
        turret.setState(Turret.State.LEFT);
        slides.setState(Slides.State.MID_DROP);
        fourbar.setState(vfourb.State.ALIGN_POSITION);
    }
    public void intake()
    {
        fourbar.setState(vfourb.State.PRIMED);
        intake.setState(Intake.State.INTAKING);
        waitSec(0.5);
    }
    public void intakeLow()
    {
        fourbar.setState(vfourb.State.STACK_LOW);
        intake.setState(Intake.State.INTAKING);
        waitSec(0.5);
    }
    public void deposit()
    {
        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        waitSec(0.4);
        this.intake.setState(Intake.State.DEPOSITING);
        waitSec(0.3);
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
