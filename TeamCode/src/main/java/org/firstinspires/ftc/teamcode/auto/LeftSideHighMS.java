package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.RobotTemp.odomServoPos;
import static org.firstinspires.ftc.teamcode.RobotTemp.sideOdomServoPos;
import static org.firstinspires.ftc.teamcode.modules.ground.GroundIntake.State.DEPOSITING;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotTemp;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.transfer.vfourb;
import org.firstinspires.ftc.teamcode.modules.turret.AlignerAuto;
import org.firstinspires.ftc.teamcode.modules.turret.Detector;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.vision.Camera;
import org.firstinspires.ftc.teamcode.pipelines.colorDetection;
import org.firstinspires.ftc.teamcode.util.BackgroundCR;
import org.firstinspires.ftc.teamcode.util.Left;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.channels.Pipe;

@Autonomous
@Left
public class LeftSideHighMS extends LinearOpMode {
    ElapsedTime t;
    RobotTemp robot;
    Intake intake;
    Slides slides;
    Claw claw;
    Deposit deposit;
    GroundIntake groundIntake;
    Turret turret;
    TelemetryPacket packet;
    Detector detector1;
    OpenCvWebcam camera, camera2;
    colorDetection pipeline;
    Pose2d startPose = new Pose2d(38,61, Math.toRadians(270));
    double timer = 0;
    int cycle = 0;
    double state=-1;
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), viewportContainerIds[0]);
        camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 2"), viewportContainerIds[1]);
        //aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        pipeline=new colorDetection(telemetry);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        camera2.setPipeline(detector1=new Detector());
        camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera2.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });*/

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
        timer = System.currentTimeMillis();


        //telemetry.setMsTransmissionInterval(50);
        robot.sideOdo.setPosition(sideOdomServoPos);
        robot.midOdo.setPosition(odomServoPos);
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(37, 12.3),robot.getVelocityConstraint(60, 5.939, 13.44),
                        robot.getAccelerationConstraint(65))

                .addTemporalMarker(0,()->{
                    slides.setState(Slides.State.HIGH);
                    groundIntake.setState(GroundIntake.State.DEPOSITING);
                })
                .addTemporalMarker(0.15,()->{
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    turret.setState(Turret.State.LEFT_SIDE_HIGH_PRELOAD);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                })

                .build();
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .lineToLinearHeading(new Pose2d(39.5,13, Math.toRadians(6.5)))
                .addTemporalMarker(0.1,()->{
                    turret.setState(Turret.State.ZERO);
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })
                .addTemporalMarker(0.4,()->{
                    slides.setState(Slides.State.CYCLE0);
                })
                .build();
        Trajectory initIntake = robot.trajectoryBuilder(new Pose2d(39.5,13, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(56, 13),robot.getVelocityConstraint(55, 5.939, 13.44),
                        robot.getAccelerationConstraint(60))
                .addTemporalMarker(0.1, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    groundIntake.setState(GroundIntake.State.OFF);
                    deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })
                .build();

        Trajectory cycleDrop = robot.trajectoryBuilder(initIntake.end())
                .lineToConstantHeading(new Vector2d(36.1, 13),robot.getVelocityConstraint(58, 5.939, 13.44),
                        robot.getAccelerationConstraint(60))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.CYCLE_HIGH);
                })
                .addTemporalMarker(0.375, ()->{
                    turret.setState(Turret.State.LEFT_SIDE_HIGH);

                })
                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(cycleDrop.end())
                .lineToConstantHeading(new Vector2d(56, 13),robot.getVelocityConstraint(55, 5.939, 13.44),
                        robot.getAccelerationConstraint(60))
                .addTemporalMarker(0.7, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })
                .build();
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
                .lineToConstantHeading(new Vector2d(13,14)).build();
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
                .lineToConstantHeading(new Vector2d(37,14)).build();
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
                .lineToConstantHeading(new Vector2d(61,14)).build();
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
        //camera.stopStreaming();


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
            dropOff(false);
        }


        if(state==1) robot.followTrajectory(endLeft);
        else if(state==2) robot.followTrajectory(endMiddle);
        else if(state==3) robot.followTrajectory(endRight);
        robot.turn(Math.toRadians(100));
        slides.setState(Slides.State.BOTTOM);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-2000< timer){
            robot.update();
        }



    }
   public void dropOff(boolean preload){
       deposit.setExtension(Deposit.ExtensionState.EXTEND);
       //deposit.setAngle(Deposit.AngleState.VECTORING);
       timer = System.currentTimeMillis();
       if(!preload)
       {
           robot.detector2.setState(AlignerAuto.State.POLE);
           turret.setState(Turret.State.LEFT_SIDE_HIGH);
           while(System.currentTimeMillis()-100<timer)
           {

           }
       }
        turret.setState(Turret.State.AUTOALIGN);
        while(System.currentTimeMillis()-300 < timer){
            if (turret.detector.getLocation() == AlignerAuto.Location.MIDDLE&&turret.getState()==Turret.State.AUTOALIGN) {
                turret.setState(Turret.State.IDLE);
            }
            telemetry.addData("TURRET", turret.getState());
            telemetry.update();
            robot.update();
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
        while(System.currentTimeMillis()-270< timer){
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

