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
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.turret.AlignerAuto;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Left;
import org.firstinspires.ftc.teamcode.util.Right;
import org.firstinspires.ftc.teamcode.util.moduleUtil.RunCondition;
import org.firstinspires.ftc.teamcode.util.moduleUtil.Task;
import org.firstinspires.ftc.teamcode.util.moduleUtil.TaskScheduler;

@Autonomous
@Right
public class CycleTest extends LinearOpMode
{
    ElapsedTime t;
    Robot robot;
    Intake intake;
    Slides slides;
    Claw claw;
    Deposit deposit;
    GroundIntake groundIntake;
    Turret turret;
    Pose2d startPose = new Pose2d(-35.25,13.5, Math.toRadians(180));
    double timer = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new Robot(this);


        deposit = robot.deposit;
        claw = robot.claw;
        slides = robot.slides;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        slides.setState(Slides.State.BOTTOM);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        claw.setState(Claw.State.CLOSE);
        turret.setState(Turret.State.ZERO);
        timer = System.currentTimeMillis();
        TaskScheduler scheduler=new TaskScheduler(this);


        //telemetry.setMsTransmissionInterval(50);
        robot.sideOdo.setPosition(sideOdomServoPos);
        robot.midOdo.setPosition(odomServoPos);

        Trajectory initIntake = robot.trajectoryBuilder(new Pose2d(-35.25,13.5, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-54.8, 13.75),robot.getVelocityConstraint(50, 5.939, 13.44),
                        robot.getAccelerationConstraint(50))
                .addTemporalMarker(0.1, ()->{
                    //deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    groundIntake.setState(GroundIntake.State.OFF);
                    //Context.wackEnabled=true;
                    //deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })
                .build();

        /*Trajectory cycleDrop = robot.trajectoryBuilder(new Pose2d(-56.8, 13.75, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-36.4, 13.75),robot.getVelocityConstraint(30, 5.939, 13.44),
                        robot.getAccelerationConstraint(20))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.CYCLE_HIGH);
                    //claw.setPoleState(Claw.Pole.DOWN);
                })
                .addTemporalMarker(0.3, ()->{
                    turret.setState(Turret.State.RIGHT_SIDE_HIGH);
                    RunCondition r=new RunCondition(()->Math.abs(turret.encoder.getCurrentPosition()+2775)<150);
                    scheduler.scheduleTask(turret.task(Turret.State.AUTOALIGN, r));
                })
                .addTemporalMarker(0.5, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })
                /*.addTemporalMarker(0.6, ()->
                {
                    turret.setState(Turret.State.AUTOALIGN);
                })*/
                //.build();*/
        Trajectory cycleIntake = robot.trajectoryBuilder(new Pose2d(-36.4, 13.75, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-54.8, 13.75),robot.getVelocityConstraint(50, 5.939, 13.44),
                        robot.getAccelerationConstraint(50))
                .addDisplacementMarker(16, ()->
                {
                    robot.localizer.setMode(Localizer.LocalizationMode.TO_INTAKE);
                })
                .addTemporalMarker(0.0, ()->{
                    turret.setState(Turret.State.ZERO);
                })
                /*.addTemporalMarker(0.7, ()->{
                    //deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })*/
                .build();



                robot.setPoseEstimate(startPose);
                claw.setState(Claw.State.OPEN);
                t=new ElapsedTime();

                waitForStart();

                for(int i = 0; i < 5; i++)
                {
                    turret.setState(Turret.State.ZERO);
                    if(i==1) slides.setState(Slides.State.CYCLE1);
                    else if(i==2) slides.setState(Slides.State.CYCLE2);
                    else if(i==3) slides.setState(Slides.State.CYCLE3);
                    else if(i==4) slides.setState(Slides.State.CYCLE4);
                    //claw.setPoleState(Claw.Pole.UP);
                    if(i==0) robot.localizer.setMode(Localizer.LocalizationMode.TO_INTAKE);
                    if(i==0)robot.followTrajectory(initIntake);
                    else robot.followTrajectory(cycleIntake);
                    robot.localizer.setMode(Localizer.LocalizationMode.INTAKING);

                    robot.deposit.setExtension(Deposit.ExtensionState.HALF);
                    while(robot.localizer.frontDistL>7.9)
                    {
                        robot.setDrivePower(new Pose2d(0.1, 0, 0));
                        robot.update();
                        robot.localizer.update();
                    }
                    robot.setDrivePower(new Pose2d(0, 0, 0));
                    Pose2d endPose=robot.getPoseEstimate();



                    Trajectory cycleDrop = robot.trajectoryBuilder(endPose)
                            .lineToLinearHeading(new Pose2d(-32.85, 13.75, Math.toRadians(180)),robot.getVelocityConstraint(60, 5.939, 13.44),
                                    robot.getAccelerationConstraint(60))
                            .addTemporalMarker(0, ()->{
                                slides.setState(Slides.State.CYCLE_HIGH);
                                //claw.setPoleState(Claw.Pole.DOWN);
                            })
                            .addTemporalMarker(0.3, ()->{
                                turret.setState(Turret.State.RIGHT_SIDE_HIGH);
                                RunCondition r=new RunCondition(()->Math.abs(turret.encoder.getCurrentPosition()+2775)<300);
                                scheduler.scheduleTask(turret.task(Turret.State.AUTOALIGN, r));
                            })
                            .addTemporalMarker(0.5, ()->{
                                deposit.setExtension(Deposit.ExtensionState.EXTEND);
                            }).build();

                    //robot.followTrajectory(intakeTrajectory());
                    intake();
                    robot.localizer.setMode(Localizer.LocalizationMode.COMPLETED);

                    robot.followTrajectory(cycleDrop);
                    dropOff(false);
                }
    }

    public void dropOff(boolean preload){
        //claw.setPoleState(Claw.Pole.DEPOSIT);
        deposit.setExtension(Deposit.ExtensionState.HALF);
        deposit.setAngle(Deposit.AngleState.VECTORING);
        t.reset();
        //timer = System.currentTimeMillis();
        /*while(System.currentTimeMillis()-200<timer)
        {
            robot.update();
        }*/
        if(!preload)
        {
            turret.setState(Turret.State.AUTOALIGN);
            while(t.milliseconds()<300){
                /*if (turret.autoalign.getLocation() == AlignerAuto.Location.MIDDLE&&turret.getState()==Turret.State.AUTOALIGN) {
                    turret.setState(Turret.State.IDLE);
                }*/
                telemetry.addData("TURRET", turret.getState());
                telemetry.update();
                robot.update();
            }
        }
        else
        {
            while (t.milliseconds()<400)
            {

            }
        }

        /*while(slides.isBusy())
        {
        }*/

        claw.setState(Claw.State.OPEN_WIDE);
        turret.setState(Turret.State.IDLE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-125<timer){
            robot.update();
        }

        //claw.setPoleState(Claw.Pole.DOWN);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);

        turret.setState(Turret.State.ZERO);
        //claw.setPoleState(Claw.Pole.UP);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-75< timer){
            robot.update();
        }
    }

    public void intake(){
        t.reset();

        deposit.setExtension(Deposit.ExtensionState.EXTEND);

        while(t.milliseconds()<300)
        {

        }

        /*while(System.currentTimeMillis()-100< timer){
            robot.update();
        }*/
        /*if(cycleNum<2)
        {
            robot.detector2.setState(AlignerAuto.State.CONESTACK);
            timer = System.currentTimeMillis();
            turret.setState(Turret.State.AUTOALIGN);
            while(System.currentTimeMillis()-150<timer && robot.detector2.getLocation()!=AlignerAuto.Location.MIDDLE)
            {
            }
            turret.setState(Turret.State.IDLE);
        }*/

        claw.setState(Claw.State.CLOSE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-225< timer){
            robot.update();
        }
        //deposit.setExtension(Deposit.ExtensionState.RETRACT);
        slides.setState(Slides.State.INTAKE_AUTO);
        deposit.setAngle(Deposit.AngleState.VECTORING);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-50 < timer){
            robot.update();
        }



        deposit.setExtension(Deposit.ExtensionState.RETRACT);
    }
}
