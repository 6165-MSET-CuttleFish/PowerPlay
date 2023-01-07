package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.Slides.Slides;
import org.firstinspires.ftc.teamcode.modules.Transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.detection.visionProcessing.Detector;
import org.firstinspires.ftc.teamcode.modules.Turret.Turret;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.detection.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class LeftSideAuto extends LinearOpMode {
    ElapsedTime t;
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(38,61,Math.toRadians(270));
    double timer = 0;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double intakeY = 11;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    int x = 1;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    @Override

    public void runOpMode() throws InterruptedException {
        t=new ElapsedTime();
        robot = new Robot(this);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        fourbar.setState(vfourb.State.STACK_PRIMED);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-200 < timer){}
        turret.setState(Turret.State.INIT);

        robot.alignUp();
        //turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fourbar.setState(vfourb.State.INTAKE_POSITION);
        //camInit();
        /*
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-38,40))
                .build();*/

        //MOVE TO MID JUNCTION, ACTUATE AND DEPOSIT ON MID JUNCTION
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(36.5, 20),robot.getVelocityConstraint(42.5, 5.939, 13.44),
                        robot.getAccelerationConstraint(50))
                .addDisplacementMarker(1, ()->{
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    //robot.aligDownn();
                })
                .addDisplacementMarker(2.5, ()->{

                    //groundIntake.setState(GroundIntake.State.INTAKING);

                    turret.setState(Turret.State.LEFT);
                    //turret.update();
                    slides.setState(Slides.State.MID_DROP);

                })
                .addTemporalMarker(2,()->{
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);

                })
                .addTemporalMarker(2.1, ()->{
                    intake.setState(Intake.State.DEPOSITING);
                    waitSec(0.25);
                })
                .addDisplacementMarker(()->
                {
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    waitSec(0.115);
                })
                .build();

        //RESET EVERYTHING, MOVE TO STACK, READY FOR PICK UP
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .addTemporalMarker(0,()->{
                    fourbar.setState(vfourb.State.STACK_PRIMED);
                    slides.setState(Slides.State.BOTTOM);
                    turret.setState(Turret.State.ZERO);

                    //groundIntake.setState(GroundIntake.State.DEPOSITING);
                    robot.alignUp();

                })
                .lineToConstantHeading(new Vector2d(40, 8.0))
                .build();
        Trajectory preload3 = robot.trajectoryBuilder(preload2.end())

                .lineToConstantHeading(new Vector2d(40, intakeY))
                .build();
        Trajectory preload4 = robot.trajectoryBuilder(preload3.end())
                /* .addTemporalMarker(0,()->{
                     fourbar.setState(vfourb.State.STACK_PRIMED);
                     slides.setState(Slides.State.BOTTOM);
                     turret.setState(Turret.State.ZERO);
                     groundIntake.setState(GroundIntake.State.DEPOSITING);
                 })*/
                .addTemporalMarker(0.5, ()->{
                    robot.alignDown();
                })
                .lineToLinearHeading(new Pose2d(41, intakeY, Math.toRadians(0)))
                .build();
        //MOVE TO STACK, PICK UP FIRST CONE

        Trajectory initCycle = robot.trajectoryBuilder(preload4.end())
                .lineToConstantHeading(new Vector2d(67, intakeY),robot.getVelocityConstraint(25, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))


                .addDisplacementMarker(2, ()->{
                    slides.setState(Slides.State.INTAKE_AUTO);
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    intake.setState(Intake.State.OFF);
                })
                .build();

        //MOVE TO MID JUNCTION, ACTUATE AND DROP OFF FIRST CONE
        Trajectory cycleDropOff1 = robot.trajectoryBuilder(initCycle.end())

                .lineToConstantHeading(new Vector2d(27,13.5))
                .addDisplacementMarker(2.5, ()->{
                    //groundIntake.setState(GroundIntake.State.OFF);
                    turret.setState(Turret.State.RIGHT);
                    slides.setState(Slides.State.MID_DROP);
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .addDisplacementMarker(13, ()->{
                    intake.setState(Intake.State.OFF);
                })
                .build();
        Trajectory cycleDropOff2 = robot.trajectoryBuilder(initCycle.end())

                .lineToConstantHeading(new Vector2d(27.25,13.5))
                .addDisplacementMarker(2.5, ()->{
                    //groundIntake.setState(GroundIntake.State.OFF);
                    turret.setState(Turret.State.RIGHT);
                    slides.setState(Slides.State.MID_DROP);
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .addDisplacementMarker(13, ()->{
                    intake.setState(Intake.State.OFF);
                })
                .build();
        Trajectory cycleIntakePrep = robot.trajectoryBuilder(cycleDropOff1.end())
                .lineToConstantHeading(new Vector2d(45, intakeY))
                .addTemporalMarker(0, ()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                    //groundIntake.setState(GroundIntake.State.DEPOSITING);
                })

                .build();
        //MOVE TO STACK, PICK UP ANOTHER CONE
        Trajectory cycleIntakeHigh = robot.trajectoryBuilder(cycleIntakePrep.end())
                .addDisplacementMarker(0,()->{
                    robot.slides.setState(Slides.State.INTAKE_AUTO);
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    intake.setState(Intake.State.OFF);
                })

                .lineToConstantHeading(new Vector2d(67, intakeY),
                        robot.getVelocityConstraint(25, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))



                .build();
        Trajectory cycleIntakeLow = robot.trajectoryBuilder(cycleIntakePrep.end())

                .lineToConstantHeading(new Vector2d(67,intakeY),robot.getVelocityConstraint(25, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))

                .addTemporalMarker(0, ()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                    //groundIntake.setState(GroundIntake.State.DEPOSITING);
                })
                .addDisplacementMarker(15,()->{
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    intake.setState(Intake.State.OFF);
                })
                .build();
        Trajectory endLeft = robot.trajectoryBuilder(cycleDropOff2.end())
                .addTemporalMarker(0,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })
                .addTemporalMarker(0.1,()->{
                    fourbar.setState(vfourb.State.VERTICAL);
                    //intake.setState(Intake.State.OFF);
                })
                .lineToConstantHeading(new Vector2d(17,11.98)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDropOff2.end())
                .addTemporalMarker(0,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })
                .addTemporalMarker(0.1,()->{
                    fourbar.setState(vfourb.State.VERTICAL);
                    //intake.setState(Intake.State.OFF);
                })
                .lineToConstantHeading(new Vector2d(38,11.98)).build();
        Trajectory endRight = robot.trajectoryBuilder(cycleDropOff1.end())
                .addTemporalMarker(0,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })
                .addTemporalMarker(0.1,()->{
                    fourbar.setState(vfourb.State.VERTICAL);
                    //intake.setState(Intake.State.OFF);
                })
                .lineToConstantHeading(new Vector2d(55,11.98)).build();
        /*Trajectory cycleIntake = robot.trajectoryBuilder(preload3.end())
                        .lineToConstantHeading(new Vector2d(-48,10))
                                .build();*/
        //robot.thread.run();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        //dashboard.startCameraStream(webcam, 30);
        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        TelemetryPacket packet =new TelemetryPacket();
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            telemetry.addLine(String.format("detections", currentDetections.size()));
            if(currentDetections.size()>0) {
                tagToTelemetry(currentDetections.get(0));
                tagOfInterest = currentDetections.get(0);
            }
            //dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {

            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {

            if(tagOfInterest.id == 4)
            {
                x =1;
            }
            else if(tagOfInterest.id == 7)
            {
                x=2;
            }
            else if(tagOfInterest.id == 8)
            {
                x=3;
            }
        }
        if (isStopRequested()) return;
        timer = System.currentTimeMillis();
        //preload
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
        robot.followTrajectory(preload2);
        robot.followTrajectory(preload3);
        robot.followTrajectory(preload4);
        groundIntake.setState(GroundIntake.State.OFF);
        //robot.turn(Math.toRadians(-100));
        //1st cycle
        robot.followTrajectory(initCycle);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();
        //2nd cycle
        robot.followTrajectory(cycleIntakePrep);
        robot.followTrajectory(cycleIntakeHigh);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();
        //3rd cycle
        robot.followTrajectory(cycleIntakePrep);
        robot.followTrajectory(cycleIntakeLow);
        cycleIntake();
        robot.followTrajectory(cycleDropOff2);
        cycleDeposit();
        //picking up 4th cone?
        /*
        robot.followTrajectory(cycleIntakePrep);
        robot.followTrajectory(cycleIntakeLow);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();*/
        //park
        if(x == 1){
            robot.followTrajectory(endLeft);
            robot.alignUp();
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis()-150 < timer){}
        }
        else if( x==2){
            robot.followTrajectory(cycleIntakePrep);
            robot.followTrajectory(cycleIntakeLow);
            cycleIntake();
        }
        else if (x==3){
            robot.followTrajectory(endMiddle);
            robot.alignUp();
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis()-150 < timer){}
        }
        //4th cycle
        /*
        robot.followTrajectory(cycleIntakeLow);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();
        robot.followTrajectory(cycleIntakeLow);*/
    }
    //if(isStopRequested()) return;


    private void cycleIntake(){

        //timer = System.currentTimeMillis();
        robot.intake.setState(Intake.State.INTAKING);
        robot.slides.setState(Slides.State.BOTTOM);
        //while(System.currentTimeMillis()- < timer){}
        //robot.groundIntake.setState(GroundIntake.State.DEPOSITING);
        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);

        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-400 < timer){}
        //robot.intake.setState(Intake.State.OFF);
        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-40 < timer){}
    }
    private void cycleDeposit(){
        robot.fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        timer = System.currentTimeMillis();

        while(System.currentTimeMillis()-110< timer){
            //turret.autoAlign();
        }
        intake.setState(Intake.State.DEPOSITING);
        timer = System.currentTimeMillis();

        while(System.currentTimeMillis()-300< timer){
            //turret.autoAlign();
        }
        robot.fourbar.setState(vfourb.State.STACK_PRIMED);
    }
    /*
    public void camInit() {
        final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
        final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector1 = new Detector());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                System.out.println("START");
            }
            public void onError(int errorCode) {
            }
        });
        //dashboard.startCameraStream(webcam, 30);
        telemetry.addLine("waiting for start");
        telemetry.update();
    }*/
    public void waitSec(double seconds)
    {
        t.reset();
        while(t.milliseconds()<seconds*1000)
        {
            //stall
        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
