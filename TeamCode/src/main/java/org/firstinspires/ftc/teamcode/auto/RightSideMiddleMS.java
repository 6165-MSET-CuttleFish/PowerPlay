package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.RobotTemp.odomServoPos;
import static org.firstinspires.ftc.teamcode.RobotTemp.sideOdomServoPos;

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
import org.firstinspires.ftc.teamcode.modules.turret.Detector;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.vision.Camera;
import org.firstinspires.ftc.teamcode.pipelines.colorDetection;
import org.firstinspires.ftc.teamcode.util.BackgroundCR;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RightSideMiddleMS extends LinearOpMode {
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
    OpenCvWebcam camera;
    colorDetection pipeline;
    Pose2d startPose = new Pose2d(-38,61,Math.toRadians(270));
    double timer = 0;
    int cycle = 0;
    double state=-1;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

        robot = new RobotTemp(this);


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
                .lineToConstantHeading(new Vector2d(-36, 12))

                .addTemporalMarker(0,()->{
                    slides.setState(Slides.State.MID);
                    groundIntake.setState(GroundIntake.State.DEPOSITING);
                })
                .addTemporalMarker(0.4,()->{
                    turret.setState(Turret.State.RIGHT_SIDE_HIGH);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                })

                .build();
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                        .lineToLinearHeading(new Pose2d(-35.25,11, Math.toRadians(180)))
                .addTemporalMarker(0.1,()->{
                    turret.setState(Turret.State.ZERO);
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })
                .addTemporalMarker(0.4,()->{
                    slides.setState(Slides.State.CYCLE0);
                })
                                .build();
        Trajectory initIntake = robot.trajectoryBuilder(preload2.end())
                .lineToConstantHeading(new Vector2d(-57, 11),robot.getVelocityConstraint(50, 5.939, 13.44),
                        robot.getAccelerationConstraint(50))
                .addTemporalMarker(0.1, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    groundIntake.setState(GroundIntake.State.OFF);
                })
                .build();

        Trajectory cycleDrop = robot.trajectoryBuilder(initIntake.end())
                .lineToConstantHeading(new Vector2d(-36.5, 12))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.MID);
                })
                .addTemporalMarker(0.2, ()->{
                    turret.setState(Turret.State.RIGHT_SIDE_MID);
                })
                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(cycleDrop.end())
                .lineToConstantHeading(new Vector2d(-57, 11),robot.getVelocityConstraint(50, 5.939, 13.44),
                        robot.getAccelerationConstraint(50))
                .addTemporalMarker(0.1, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);

                })
                .build();
        Trajectory endLeft = robot.trajectoryBuilder(cycleDrop.end())

                .lineToConstantHeading(new Vector2d(-13,13)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDrop.end())

                .lineToConstantHeading(new Vector2d(-40,13)).build();
        Trajectory endRight = robot.trajectoryBuilder(cycleDrop.end())

                .lineToConstantHeading(new Vector2d(-60,11)).build();

        while(!isStarted()&&!isStopRequested())
        {
            double tempState=pipeline.getOutput();
            telemetry.addData("State: ", tempState);
            telemetry.addData("H Value", pipeline.getHAvg());
            telemetry.update();
            if(tempState>0)
            {
                state=tempState;
            }
        }

        waitForStart();


        if (isStopRequested()) return;
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        dropOff();
        robot.followTrajectory(preload2);

        for(int i = 0; i < 5; i++){
            if(i==0)slides.setState(Slides.State.CYCLE0);
            else if (i==1)slides.setState(Slides.State.CYCLE1);
            else if (i==2)slides.setState(Slides.State.CYCLE2);
            else if (i==3)slides.setState(Slides.State.CYCLE3);
            else if (i==4)slides.setState(Slides.State.CYCLE4);
            if(i!=0)robot.followTrajectory(cycleIntake);
            else if(i==0) robot.followTrajectory(initIntake);
            intake();
            robot.followTrajectory(cycleDrop);
            dropOff();
        }
        turret.setState(Turret.State.ZERO);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        claw.setState(Claw.State.OPEN);
        slides.setState(Slides.State.BOTTOM);
        robot.followTrajectory(endRight);


    }
    public void dropOff(){

        deposit.setExtension(Deposit.ExtensionState.EXTEND);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-650 < timer){
            robot.update();
        }
        claw.setState(Claw.State.OPEN);
         timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-150< timer){
            robot.update();
        }
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        turret.setState(Turret.State.ZERO);

    }
    public void intake(){
        claw.setState(Claw.State.CLOSE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-250< timer){
            robot.update();
        }
        //deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.VECTORING);
        slides.setState(Slides.State.MID);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-250< timer){
            robot.update();
        }
deposit.setExtension(Deposit.ExtensionState.RETRACT);


    }
}