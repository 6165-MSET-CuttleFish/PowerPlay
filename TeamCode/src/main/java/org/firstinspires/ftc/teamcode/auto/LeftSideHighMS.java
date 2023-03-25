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
import org.firstinspires.ftc.teamcode.modules.turret.Detector;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.pipelines.colorDetection;
import org.firstinspires.ftc.teamcode.util.Left;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Left
public class LeftSideHighMS extends LinearOpMode {
    ElapsedTime t;
    Robot robot;
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
    Pose2d startPose = new Pose2d(34,61, Math.toRadians(270));
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
                .lineToConstantHeading(new Vector2d(34, 10.5))

                .addTemporalMarker(0,()->{

                    groundIntake.setState(GroundIntake.State.DEPOSITING);
                })
                .addTemporalMarker(0.3,()->{
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    slides.setState(Slides.State.HIGH);

                })
                .addTemporalMarker(0.45, ()->{
                    turret.setState(Turret.State.LEFT_SIDE_HIGH_PRELOAD);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                    claw.setPoleState(Claw.Pole.DEPOSIT);
                })
                .build();
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .lineToLinearHeading(new Pose2d(34,15, Math.toRadians(0)))

                .addTemporalMarker(0.1,()->{
                    turret.setState(Turret.State.ZERO);
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);

                })
                .addTemporalMarker(0.4,()->{
                    claw.setPoleState(Claw.Pole.UP);
                    slides.setState(Slides.State.CYCLE0);
                })
                .build();
        Trajectory initIntake = robot.trajectoryBuilder(preload2.end())
                .lineToConstantHeading(new Vector2d(57.25, 15),
                        robot.getVelocityConstraint(52.5, 5.939, 16.92),
                        robot.getAccelerationConstraint(55))

                .addTemporalMarker(0.1, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    groundIntake.setState(GroundIntake.State.OFF);
                    deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })
                .build();

        Trajectory cycleDrop = robot.trajectoryBuilder(initIntake.end())
                .lineToConstantHeading(new Vector2d(32.5, 15))
                .addTemporalMarker(0.2, ()->{
                    slides.setState(Slides.State.CYCLE_HIGH);
                    claw.setPoleState(Claw.Pole.DOWN);
                })
                .addTemporalMarker(0.25, ()->{
                    turret.setState(Turret.State.LEFT_SIDE_HIGH);


                })
                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(cycleDrop.end())
                .lineToConstantHeading(new Vector2d(57.25, 15),
                        robot.getVelocityConstraint(52.5, 5.939, 16.92),
                        robot.getAccelerationConstraint(55))

                .addTemporalMarker(0.65, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    deposit.setAngle(Deposit.AngleState.AUTO_INTAKE);
                })
                .build();
        Trajectory endRight = robot.trajectoryBuilder(cycleDrop.end())

                .addTemporalMarker(0.0, ()->{
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);


                })
                .addTemporalMarker(0.2, ()->{
                    claw.setPoleState(Claw.Pole.UP);
                    slides.setState(Slides.State.BOTTOM);
                })
                .lineToConstantHeading(new Vector2d(13,14)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDrop.end())
                /*
                .addTemporalMarker(0.0, ()->{
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    claw.setState(Claw.State.OPEN);

                })
                .addTemporalMarker(0.07, ()->{
                    slides.setState(Slides.State.BOTTOM);
                })*/
                .lineToConstantHeading(new Vector2d(37,14)).build();
        Trajectory endLeft = robot.trajectoryBuilder(cycleDrop.end())
                /*
                .addTemporalMarker(0.0, ()->{
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    claw.setState(Claw.State.OPEN);

                })
                .addTemporalMarker(0.2, ()->{
                    slides.setState(Slides.State.BOTTOM);
                })*/
                .lineToConstantHeading(new Vector2d(57,14)).build();
        double tempState;


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
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-100<timer)
        {
            //stall a little
        }
        for(int i = 0; i < 5; i++){
            if(i==1) slides.setState(Slides.State.CYCLE1);
            else if(i==2) slides.setState(Slides.State.CYCLE2);
            else if(i==3) slides.setState(Slides.State.CYCLE3);
            else if(i==4) slides.setState(Slides.State.CYCLE4);
            claw.setPoleState(Claw.Pole.UP);
            if(i==0)robot.followTrajectory(initIntake);
            else robot.followTrajectory(cycleIntake);
            intake();
            robot.followTrajectory(cycleDrop);
            dropOff(true);
        }
        robot.followTrajectory(endRight);



/*
        for(int i = 0; i < 4; i++){
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
        }*/
        //robot.closeCameras();
/*
        if(state==1) robot.followTrajectory(endLeft);
        else if(state==2) robot.followTrajectory(endMiddle);
        else if(state==3) robot.followTrajectory(endRight);
        //robot.turn(Math.toRadians(100));
        slides.setState(Slides.State.BOTTOM);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-2000< timer&&opModeIsActive())
        {
            robot.update();
        }*/
    }

   public void dropOff(boolean preload){
       claw.setPoleState(Claw.Pole.DEPOSIT);
       deposit.setExtension(Deposit.ExtensionState.EXTEND);
       //deposit.setAngle(Deposit.AngleState.VECTORING);
       timer = System.currentTimeMillis();
       while(System.currentTimeMillis()-265<timer)
       {
           robot.update();
       }
       if(!preload)
       {
           turret.setState(Turret.State.AUTOALIGN);
           while(System.currentTimeMillis()-400 < timer){
               if (turret.autoalign.getLocation() == AlignerAuto.Location.MIDDLE&&turret.getState()==Turret.State.AUTOALIGN) {
                   turret.setState(Turret.State.IDLE);
               }
               telemetry.addData("TURRET", turret.getState());
               telemetry.update();
               robot.update();
           }
       }
       else
       {

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

