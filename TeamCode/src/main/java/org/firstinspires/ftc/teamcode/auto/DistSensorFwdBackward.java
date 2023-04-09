package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Robot.odomServoPos;
import static org.firstinspires.ftc.teamcode.Robot.sideOdomServoPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.relocalizer.Localizer;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.util.Left;
import org.firstinspires.ftc.teamcode.util.moduleUtil.RunCondition;

@Autonomous
@Left
public class DistSensorFwdBackward extends LinearOpMode
{
    Robot robot;
    Pose2d startPose = new Pose2d(-35.25,13.5, Math.toRadians(180));
    ElapsedTime t;

    @Override
    public void runOpMode() throws InterruptedException
    {
        t=new ElapsedTime();
        robot=new Robot(this);

        robot.sideOdo.setPosition(sideOdomServoPos);
        robot.midOdo.setPosition(odomServoPos);

        Trajectory initIntake = robot.trajectoryBuilder(new Pose2d(-35.25,13.5, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-65.8, 13.75),robot.getVelocityConstraint(30, 5.939, 13.44),
                        robot.getAccelerationConstraint(20))
                .build();

        /*Trajectory cycleDrop = robot.trajectoryBuilder(new Pose2d(-56.8, 13.75, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-36.4, 13.75),robot.getVelocityConstraint(30, 5.939, 13.44),
                        robot.getAccelerationConstraint(20))
                /*.addTemporalMarker(0.6, ()->
                {
                    turret.setState(Turret.State.AUTOALIGN);
                })*/
                //.build();*/
        Trajectory cycleIntake = robot.trajectoryBuilder(new Pose2d(-36.4, 13.75, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-65.8, 13.75),robot.getVelocityConstraint(30, 5.939, 13.44),
                        robot.getAccelerationConstraint(20))
                .addDisplacementMarker(16, ()->
                {
                    robot.localizer.setMode(Localizer.LocalizationMode.TO_INTAKE);
                })
                /*.addTemporalMarker(0.7, ()->{
                    //deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })*/
                .build();

                Trajectory intake=robot.trajectoryBuilder(new Pose2d(-56.8, 13.75, Math.toRadians(180))).
                        lineToConstantHeading(new Vector2d(-57.5, 13.75))
                        .addTemporalMarker(0, ()->
                        {
                            robot.deposit.setExtension(Deposit.ExtensionState.EXTEND);
                        })
                                .build();

        robot.setPoseEstimate(startPose);

        waitForStart();

        for(int i = 0; i < 5; i++)
        {
            robot.claw.setState(Claw.State.OPEN_WIDE);
            robot.deposit.setExtension(Deposit.ExtensionState.RETRACT);
            if(i==0) robot.localizer.setMode(Localizer.LocalizationMode.TO_INTAKE);
            if(i==0)robot.followTrajectory(initIntake);
            else robot.followTrajectory(cycleIntake);
            robot.localizer.setMode(Localizer.LocalizationMode.INTAKING);



            robot.deposit.setExtension(Deposit.ExtensionState.HALF);
            while(robot.localizer.frontDistL>7.9)
            {
                robot.setDrivePower(new Pose2d(0.2, 0, 0));
                robot.update();
            }
            robot.setDrivePower(new Pose2d(0, 0, 0));
            Pose2d endPose=robot.getPoseEstimate();

            Trajectory cycleDrop = robot.trajectoryBuilder(endPose)
                    .lineToConstantHeading(new Vector2d(-36.4, 13.75),robot.getVelocityConstraint(30, 5.939, 13.44),
                            robot.getAccelerationConstraint(20))
                    /*.addTemporalMarker(0.6, ()->
                    {
                        turret.setState(Turret.State.AUTOALIGN);
                    })*/
                    .build();

            //robot.followTrajectory(intakeTrajectory());
            intake();
            robot.localizer.setMode(Localizer.LocalizationMode.COMPLETED);

            robot.localizer.setMode(Localizer.LocalizationMode.OFF);
            robot.followTrajectory(cycleDrop);
        }
    }

    public Trajectory intakeTrajectory()
    {
        Trajectory intake=robot.trajectoryBuilder(new Pose2d(-56.8, 13.75, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-56.8+(robot.localizer.savedDist-8), 13.75, Math.toRadians(180)), robot.getVelocityConstraint(20, 5.939, 13.44),
                        robot.getAccelerationConstraint(10))
                .addTemporalMarker(0, ()->
                {
                    robot.deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })
                .build();
        return intake;
    }

    public void intake()
    {
        //robot.deposit.setExtension(Deposit.ExtensionState.HALF);

        t.reset();

        while(t.milliseconds()<300)
        {

        }

        robot.claw.setState(Claw.State.CLOSE);
    }
}
