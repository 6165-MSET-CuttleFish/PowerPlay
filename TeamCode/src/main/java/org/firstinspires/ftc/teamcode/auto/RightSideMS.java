package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotTemp;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.transfer.vfourb;
import org.firstinspires.ftc.teamcode.modules.turret.Detector;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RightSideMS extends LinearOpMode {
    ElapsedTime t;
    RobotTemp robot;
    Intake intake;
    Slides slides;
    Claw claw;
    Deposit deposit;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(-38,61,Math.toRadians(270));
    double timer = 0;
    int cycle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotTemp(this);
        intake = robot.intake;
        slides = robot.slides;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        claw = robot.claw;
        deposit = robot.deposit;
        slides.setState(Slides.State.BOTTOM);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        claw.setState(Claw.State.CLOSE);
        turret.setState(Turret.State.ZERO);

        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-38, 10))

                .addTemporalMarker(0,()->{
                    slides.setState(Slides.State.HIGH);})
                .addTemporalMarker(0.4,()->{
                    turret.setState(Turret.State.RIGHT_SIDE_HIGH);
                })

                .build();
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                        .lineToLinearHeading(new Pose2d(-35.5,8.5, Math.toRadians(180)))
                .addTemporalMarker(0,()->{

                    deposit.setAngle(Deposit.AngleState.VECTORING);
                })
                .addTemporalMarker(0.5,()->{

                })

                                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(preload2.end())
                .lineToConstantHeading(new Vector2d(-55.25, 10),robot.getVelocityConstraint(40, 5.939, 13.44),
                        robot.getAccelerationConstraint(40))

                .addTemporalMarker(0,()->{

                })
                .addTemporalMarker(0.1,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.CYCLE0);
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);

                })

                        .build();
        waitForStart();
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        robot.followTrajectory(preload2);
        dropOff();
        robot.followTrajectory(cycleIntake);
        intake();
    }
    public void dropOff(){

        deposit.setExtension(Deposit.ExtensionState.EXTEND);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-1000 < timer){}
        claw.setState(Claw.State.OPEN);
         timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-350< timer){}
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);


    }
    public void intake(){

        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-750 < timer){}
        claw.setState(Claw.State.CLOSE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-350< timer){}
        //deposit.setExtension(Deposit.ExtensionState.RETRACT);
        //deposit.setAngle(Deposit.AngleState.INTAKE);
        //turret.setState(Turret.State.ZERO);


    }
}
