package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Robot.odomServoPos;
import static org.firstinspires.ftc.teamcode.Robot.sideOdomServoPos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.turret.AlignerAuto;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Right;

@Autonomous
@Right
public class RightSideWARTIME extends LinearOpMode{
    ElapsedTime t;
    Robot robot;
    Intake intake;
    Slides slides;
    Claw claw;
    Deposit deposit;
    GroundIntake groundIntake;
    Turret turret;
    TelemetryPacket packet;
    Pose2d startPose = new Pose2d(-34,61, Math.toRadians(270));
    double timer = 0;
    int cycle = 0;
    double state=-1;
    @Override
    public void runOpMode() throws InterruptedException {


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


        //telemetry.setMsTransmissionInterval(50);
        robot.sideOdo.setPosition(sideOdomServoPos);
        robot.midOdo.setPosition(odomServoPos);
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-34, 0))

                .addTemporalMarker(0, () -> {


                })
                .addTemporalMarker(0.2, () -> {
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    slides.setState(Slides.State.CYCLE_HIGH);

                })
                .addTemporalMarker(0.3, () -> {
                    groundIntake.setState(GroundIntake.State.DEPOSITING);
                    turret.setState(Turret.State.RIGHT);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                    claw.setPoleState(Claw.Pole.DOWN);
                    deposit.setExtension(Deposit.ExtensionState.HALF);
                })
                .build();
        Trajectory preloadShift = robot.trajectoryBuilder(preload1.end())
                .lineToConstantHeading(new Vector2d(-34, 9.5))

                .addTemporalMarker(0, () -> {


                })
                .build();
        Trajectory preload2 = robot.trajectoryBuilder(preloadShift.end())
                .lineToLinearHeading(new Pose2d(-34, 13.5, Math.toRadians(180)))

                .addTemporalMarker(0.0, () -> {
                    turret.setState(Turret.State.ZERO);
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);

                })
                .addTemporalMarker(0.65, () -> {

                    slides.setState(Slides.State.CYCLE0);
                })
                .addTemporalMarker(0.8, () -> {
                    claw.setPoleState(Claw.Pole.UP);
                })
                .build();
        Trajectory initIntake = robot.trajectoryBuilder(preload2.end())
                .lineToConstantHeading(new Vector2d(-55.5, 13.5),
                        robot.getVelocityConstraint(40, 5.939, 16.92),
                        robot.getAccelerationConstraint(42.5))

                .addTemporalMarker(0.1, () -> {
                    //deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    groundIntake.setState(GroundIntake.State.OFF);
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })
                .build();

        Trajectory cycleDrop = robot.trajectoryBuilder(initIntake.end())
                .lineToConstantHeading(new Vector2d(-32.85, 12.7),
                        robot.getVelocityConstraint(52.5, 5.939, 16.92),
                        robot.getAccelerationConstraint(50))
                .addTemporalMarker(0.1, () -> {
                    slides.setState(Slides.State.CYCLE_HIGH);
                    claw.setPoleState(Claw.Pole.DOWN);
                })
                .addTemporalMarker(0.25, () -> {
                    turret.setState(Turret.State.RIGHT_SIDE_HIGH);


                })
                .addTemporalMarker(0.5, () -> {
                    deposit.setExtension(Deposit.ExtensionState.HALF);
                })
                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(cycleDrop.end())
                .lineToConstantHeading(new Vector2d(-55.5, 13.5),
                        robot.getVelocityConstraint(40, 5.939, 16.92),
                        robot.getAccelerationConstraint(42.5))

                .addTemporalMarker(0.0, () -> {
                    turret.setState(Turret.State.ZERO);

                    deposit.setAngle(Deposit.AngleState.INTAKE);
                })
                .addTemporalMarker(0.3, () -> {
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })
                .build();
        Trajectory endLeft = robot.trajectoryBuilder(cycleDrop.end())

                .addTemporalMarker(0.0, () -> {
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);


                })
                .addTemporalMarker(0.2, () -> {
                    claw.setPoleState(Claw.Pole.UP);
                    slides.setState(Slides.State.BOTTOM);
                })
                .lineToConstantHeading(new Vector2d(-13, 14)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDrop.end())
                .addTemporalMarker(0.0, () -> {
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);


                })
                .addTemporalMarker(0.2, () -> {
                    claw.setPoleState(Claw.Pole.UP);
                    slides.setState(Slides.State.BOTTOM);
                })
                .lineToConstantHeading(new Vector2d(-37, 14)).build();
        Trajectory endRight = robot.trajectoryBuilder(cycleDrop.end())

                .addTemporalMarker(0.0, () -> {
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);


                })
                .addTemporalMarker(0.2, () -> {
                    claw.setPoleState(Claw.Pole.UP);
                    slides.setState(Slides.State.BOTTOM);
                })
                .lineToConstantHeading(new Vector2d(-59, 13)).build();


        waitForStart();


        if (isStopRequested()) return;
        //robot.autoCamera.pauseViewport();
        //camera.stopStreaming();


        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        /*timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-500<timer)
        {
            //stall a little
        }*/
        dropOff(true);
        robot.followTrajectory(preload2);

        for (int i = 0; i < 5; i++) {
            turret.setState(Turret.State.ZERO);
            if (i == 1) slides.setState(Slides.State.CYCLE1);
            else if (i == 2) slides.setState(Slides.State.CYCLE2);
            else if (i == 3) slides.setState(Slides.State.CYCLE3);
            else if (i == 4) slides.setState(Slides.State.CYCLE4);
            claw.setPoleState(Claw.Pole.UP);
            if (i == 0) robot.followTrajectory(initIntake);
            else robot.followTrajectory(cycleIntake);
            intake();
            robot.followTrajectory(cycleDrop);
            dropOff(false);
        }


        if (Context.signalSleeveZone == 1) robot.followTrajectory(endLeft);
        else if (Context.signalSleeveZone == 2) robot.followTrajectory(endMiddle);
        else if (Context.signalSleeveZone == 3) robot.followTrajectory(endRight);

    }

    public void dropOff(boolean preload){
        claw.setPoleState(Claw.Pole.DEPOSIT);
        deposit.setExtension(Deposit.ExtensionState.EXTEND);
        if(!preload)
        {

            timer = System.currentTimeMillis();
            while(System.currentTimeMillis()-195 < timer){
                turret.setState(Turret.State.AUTOALIGN);
                turret.update();
                robot.update();
            }
        }
        else
        {
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis()-185<timer)
            {
                robot.update();
            }
        }

        /*while(slides.isBusy())
        {

        }*/

        claw.setState(Claw.State.OPEN);
        turret.setState(Turret.State.IDLE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-115<timer){
            robot.update();
        }
        claw.setPoleState(Claw.Pole.DOWN);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);


        //claw.setPoleState(Claw.Pole.UP);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-198< timer){
            robot.update();
        }
    }
    public void intake(){
        timer = System.currentTimeMillis();
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
        while(System.currentTimeMillis()-185< timer){
            robot.update();
        }
        //deposit.setExtension(Deposit.ExtensionState.RETRACT);
        slides.setState(Slides.State.INTAKE_AUTO);
        deposit.setAngle(Deposit.AngleState.VECTORING);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-5< timer){
            robot.update();
        }



        deposit.setExtension(Deposit.ExtensionState.RETRACT);
    }
}

