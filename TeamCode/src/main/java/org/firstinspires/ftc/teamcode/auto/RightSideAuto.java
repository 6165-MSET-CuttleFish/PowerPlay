package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
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
    double timer = 0;
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

                .lineToConstantHeading(new Vector2d(-35.5, 21.5))
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
                .lineToConstantHeading(new Vector2d(-40, 11.5))

                .build();

        Trajectory initCycle = robot.trajectoryBuilder(new Pose2d(-40,10,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-63.5,12))
                .addDisplacementMarker(2, ()->{
                    slides.setState(Slides.State.LOW_DROP);
                    groundIntake.setState(GroundIntake.State.OFF);
                    intake.setState(Intake.State.INTAKING);
                })
                        .build();
        Trajectory cycleDropOff1 = robot.trajectoryBuilder(initCycle.end())
                .lineToConstantHeading(new Vector2d(-24.25,12.5),
                        robot.getVelocityConstraint(30, 5.939, 14.48),
                        robot.getAccelerationConstraint(23))
                .addDisplacementMarker(2, ()->{

                    turret.setState(Turret.State.LEFT);
                    slides.setState(Slides.State.MID_DROP);
                    //fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .build();
        Trajectory cycleIntake1 = robot.trajectoryBuilder(cycleDropOff1.end())

                .lineToConstantHeading(new Vector2d(-63.5,12),
                        robot.getVelocityConstraint(30, 5.939, 14.48),
                        robot.getAccelerationConstraint(23))

                .addDisplacementMarker(0, ()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);
                    //groundIntake.setState(GroundIntake.State.OFF);
                    intake.setState(Intake.State.INTAKING);

                })
                .addDisplacementMarker(5, ()->{
                    slides.setState(Slides.State.LOW_DROP);
                })
                .build();
        /*Trajectory cycleIntake = robot.trajectoryBuilder(preload3.end())
                        .lineToConstantHeading(new Vector2d(-48,10))
                                .build();*/

        waitForStart();
        if(isStopRequested()) return;
        timer = System.currentTimeMillis();
        //preload
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        robot.followTrajectory(preload2);
        robot.turn(Math.toRadians(-100));
        //1st cycle
        robot.followTrajectory(initCycle);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();
        //2nd cycle
        /*
        robot.followTrajectory(cycleIntake1);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();
        //3rd cycle
        robot.followTrajectory(cycleIntake1);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();*/
        while (!isStopRequested() && opModeIsActive()) ;
    }
    private void cycleIntake(){
        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-1500 < timer){}
        fourbar.setState(vfourb.State.ALIGN_POSITION);
    }
    private void cycleDeposit(){
        robot.fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        intake.setState(Intake.State.DEPOSITING);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-375 < timer){}
        robot.fourbar.setState(vfourb.State.PRIMED);
    }
}
