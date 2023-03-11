package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.RobotTemp.odomServoPos;
import static org.firstinspires.ftc.teamcode.RobotTemp.sideOdomServoPos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.common.subtyping.qual.Bottom;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotTemp;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.turret.AlignerAuto;
import org.firstinspires.ftc.teamcode.modules.turret.Detector;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.pipelines.colorDetection;
import org.firstinspires.ftc.teamcode.util.Right;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//@Autonomous
@Right
public class RightSideHighMidMH extends LinearOpMode{
    ElapsedTime t;
    RobotTemp robot;
    Intake intake;
    Slides slides;
    Claw claw;
    Deposit deposit;
    GroundIntake groundIntake;
    Turret turret;
    TelemetryPacket packet;
    Pose2d startPose = new Pose2d(-38,61, Math.toRadians(270));
    double timer = 0;
    int cycle = 0;
    double state=-1;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotTemp(this, true);



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
        turret.setState(Turret.Hall.OFF);
        timer = System.currentTimeMillis();


        telemetry.addData("Checkpoint", "2");
        telemetry.update();

        //telemetry.setMsTransmissionInterval(50);
        robot.sideOdo.setPosition(sideOdomServoPos);
        robot.midOdo.setPosition(odomServoPos);
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36.5, 13.5), robot.getVelocityConstraint(65, 5.939, 13.44),
                        robot.getAccelerationConstraint(68))

                .addTemporalMarker(0,()->{
                    slides.setState(Slides.State.HIGH);

                })
                .addTemporalMarker(0.15,()->{
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    turret.setState(Turret.State.RIGHT_SIDE_HIGH_PRELOAD);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                })

                .build();
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .lineToLinearHeading(new Pose2d(-35.75,13.75, Math.toRadians(174.5)))
                .addTemporalMarker(0.1,()->{
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })
                .addTemporalMarker(0.4,()->{
                    slides.setState(Slides.State.CYCLE0);
                    turret.setState(Turret.State.ZERO);
                })
                .build();
        Trajectory initIntake = robot.trajectoryBuilder(new Pose2d(-35.25,13.5, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-56.8, 13.75),robot.getVelocityConstraint(57, 5.939, 13.44),
                        robot.getAccelerationConstraint(59))
                .addTemporalMarker(0.1, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    groundIntake.setState(GroundIntake.State.OFF);
                    //deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })
                .build();

        Trajectory cycleDrop = robot.trajectoryBuilder(initIntake.end())
                .lineToConstantHeading(new Vector2d(-36.4, 13.75),robot.getVelocityConstraint(57, 5.939, 13.44),
                        robot.getAccelerationConstraint(57))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.CYCLE_HIGH);
                })
                .addTemporalMarker(0.375, ()->{
                    turret.setState(Turret.State.RIGHT_SIDE_HIGH);
                })

                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(cycleDrop.end())
                .lineToConstantHeading(new Vector2d(-56.8, 13.75),robot.getVelocityConstraint(57, 5.939, 13.44),
                        robot.getAccelerationConstraint(59))
                .addTemporalMarker(0.7, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })
                .build();
        Trajectory endLeft = robot.trajectoryBuilder(cycleDrop.end())
                .addTemporalMarker(0.0, ()->{
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    claw.setState(Claw.State.OPEN);

                })
                .addTemporalMarker(0.2, ()->{
                    slides.setState(Slides.State.BOTTOM);
                })
                .lineToConstantHeading(new Vector2d(-13,14)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDrop.end())
                .addTemporalMarker(0.0, ()->{
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    claw.setState(Claw.State.OPEN);

                })
                .addTemporalMarker(0.07, ()->{
                    slides.setState(Slides.State.BOTTOM);
                })
                .lineToConstantHeading(new Vector2d(-37,14)).build();
        Trajectory endRight = robot.trajectoryBuilder(cycleDrop.end())
                .addTemporalMarker(0.0, ()->{
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    claw.setState(Claw.State.OPEN);

                })
                .addTemporalMarker(0.2, ()->{
                    slides.setState(Slides.State.BOTTOM);
                })
                .lineToConstantHeading(new Vector2d(-61,14)).build();

        telemetry.addData("Checkpoint", "3");
        telemetry.update();
        double tempState;
        while(!isStarted()&&!isStopRequested())
        {
            tempState=robot.pipeline.getOutput();
            telemetry.addData("Camera 1: ", tempState);
            telemetry.addData("Autoalign Camera: ", robot.detector2.recording);
            telemetry.update();

            if(tempState>0)
            {
                state=tempState;
            }
        }
        telemetry.addData("AUTO READY", 1);
        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;
        robot.autoCamera.pauseViewport();
        //robot.autoCamera.stopStreaming();


        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        dropOff(true);
        robot.followTrajectory(preload2);

        for(int i = 0; i < 5; i++){
            //if(state==3 && i==4) break;
            if(i==0)slides.setState(Slides.State.CYCLE0);
            else if (i==1)slides.setState(Slides.State.CYCLE1);
            else if (i==2)slides.setState(Slides.State.CYCLE2);
            else if (i==3)slides.setState(Slides.State.CYCLE3);
            else if (i==4)slides.setState(Slides.State.CYCLE4);
            if(i!=0)robot.followTrajectory(cycleIntake);
            else if(i==0) robot.followTrajectory(initIntake);
            intake(i);
            robot.followTrajectory(cycleDrop);
            if(i!=3)
            {
                dropOff(false);
            }

        }
        //robot.closeCameras();


        if(state==1) robot.followTrajectory(endLeft);
        else if(state==2) robot.followTrajectory(endMiddle);
        else if(state==3) robot.followTrajectory(endRight);

        slides.setState(Slides.State.BOTTOM);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-2000< timer&&opModeIsActive()){
            robot.update();
        }
    }
    public void dropOff(boolean preload){
        //deposit.setAngle(Deposit.AngleState.VECTORING);
        deposit.setExtension(Deposit.ExtensionState.EXTEND);
        timer = System.currentTimeMillis();

        if(!preload)
        {
            turret.setState(Turret.State.AUTOALIGN);
            while(System.currentTimeMillis()-300 < timer){
                if (turret.detector.getLocation() == AlignerAuto.Location.MIDDLE&&turret.getState()==Turret.State.AUTOALIGN) {
                    turret.setState(Turret.State.IDLE);
                }
                telemetry.addData("TURRET", turret.getState());
                telemetry.update();
                robot.update();
            }
        }
        else
        {
            while(System.currentTimeMillis()-200<timer)
            {
                //stall a little
            }
        }

        /*while(slides.isBusy())
        {

        }*/

        claw.setState(Claw.State.OPEN_WIDE);
        turret.setState(Turret.State.IDLE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-225<timer){
            robot.update();
        }

        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);

        turret.setState(Turret.State.ZERO);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-120< timer){
            robot.update();
        }

    }

    public void dropOff2(){

        deposit.setExtension(Deposit.ExtensionState.EXTEND);
        timer = System.currentTimeMillis();
        turret.setState(Turret.State.AUTOALIGN);
        while(System.currentTimeMillis()-300 < timer){
            if (turret.detector.getLocation() == AlignerAuto.Location.MIDDLE&&turret.getState()==Turret.State.AUTOALIGN) {
                turret.setState(Turret.State.IDLE);
            }
            telemetry.addData("TURRET", turret.getState());
            telemetry.update();
            robot.update();
        }
        claw.setState(Claw.State.OPEN);
        turret.setState(Turret.State.IDLE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-225< timer){
            robot.update();
        }

        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);

        turret.setState(Turret.State.ZERO);
        /*timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-120< timer){
            robot.update();
        }*/
    }
    public void intake(int cycleNum){
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
        while(System.currentTimeMillis()-250< timer){
            robot.update();
        }
        //deposit.setExtension(Deposit.ExtensionState.RETRACT);
        slides.setState(Slides.State.HIGH);

        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-200< timer){
            robot.update();
        }

        deposit.setAngle(Deposit.AngleState.VECTORING);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
    }
}

