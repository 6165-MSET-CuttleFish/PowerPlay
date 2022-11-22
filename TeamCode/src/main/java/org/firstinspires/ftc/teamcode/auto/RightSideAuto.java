package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
public class RightSideAuto extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(-38,61,Math.toRadians(270));

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
        /*
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-38,40))
                .build();*/
        Trajectory preload1 = robot.trajectoryBuilder(startPose)

                .lineToConstantHeading(new Vector2d(-35, 22.25))
                        .addDisplacementMarker(2, ()->{
                            groundIntake.setState(GroundIntake.State.DEPOSITING);
                            turret.setState(Turret.State.RIGHT);
                            slides.setState(Slides.State.MID_DROP);
                            fourbar.setState(vfourb.State.ALIGN_POSITION);
                        })
                .addTemporalMarker(2,()->{
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                })
                .addTemporalMarker(2.1, ()->{
                    intake.setState(Intake.State.DEPOSITING);
                })

                                .build();
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .addTemporalMarker(0,()->{
                    fourbar.setState(vfourb.State.PRIMED);
                    slides.setState(Slides.State.BOTTOM);
                    turret.setState(Turret.State.ZERO);

                })
                .lineToConstantHeading(new Vector2d(-38.5, 10))

                .build();
        Trajectory preload3 = robot.trajectoryBuilder(preload2.end())
                .lineToLinearHeading(new Pose2d(-42,10,Math.toRadians(176)))
                .addDisplacementMarker(0.5, ()->{
                    intake.setState(Intake.State.OFF);
                })
                .build();
        Trajectory initCycle = robot.trajectoryBuilder(new Pose2d(-42,10,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-62,8))
                .addDisplacementMarker(2, ()->{
                    slides.setState(Slides.State.LOW_DROP);
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                })
                        .build();

        /*Trajectory cycleIntake = robot.trajectoryBuilder(preload3.end())
                        .lineToConstantHeading(new Vector2d(-48,10))
                                .build();*/

        waitForStart();
        if(isStopRequested()) return;

        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        robot.followTrajectory(preload2);
        robot.followTrajectory(preload3);
        robot.setPoseEstimate(new Pose2d(-42,10,Math.toRadians(180)));
        robot.followTrajectory(initCycle);
        //robot.updatePoseEstimate();
        //robot.followTrajectory(cycleIntake);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
