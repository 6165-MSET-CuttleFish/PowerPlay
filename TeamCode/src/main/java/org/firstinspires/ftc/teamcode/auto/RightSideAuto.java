package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class RightSideAuto extends LinearOpMode {
    ElapsedTime t;
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
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double intakeY = 12.24;
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
                .lineToConstantHeading(new Vector2d(-34.3, 21.6),robot.getVelocityConstraint(42.5, 5.939, 13.44),
                        robot.getAccelerationConstraint(50))
                .addDisplacementMarker(1, ()->{
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                    //robot.aligDownn();
                })
                .addDisplacementMarker(2, ()->{

                    //groundIntake.setState(GroundIntake.State.INTAKING);

                    turret.setState(Turret.State.RIGHT);
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
                .lineToConstantHeading(new Vector2d(-40, 8.0))
                .build();
        Trajectory preload3 = robot.trajectoryBuilder(preload2.end())

                .lineToConstantHeading(new Vector2d(-40, intakeY))
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
                .lineToLinearHeading(new Pose2d(-41, intakeY, Math.toRadians(180)))
                .build();
        //MOVE TO STACK, PICK UP FIRST CONE

        Trajectory initCycle = robot.trajectoryBuilder(preload4.end())
                .lineToConstantHeading(new Vector2d(-63.25,intakeY),robot.getVelocityConstraint(25, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))


                .addDisplacementMarker(2, ()->{
                    slides.setState(Slides.State.INTAKE_AUTO);
                    //groundIntake.setState(GroundIntake.State.INTAKING);
                    intake.setState(Intake.State.OFF);
                })
                .build();

        //MOVE TO MID JUNCTION, ACTUATE AND DROP OFF FIRST CONE
        Trajectory cycleDropOff1 = robot.trajectoryBuilder(initCycle.end())

                .lineToConstantHeading(new Vector2d(-26.7,12.75))
                .addDisplacementMarker(2, ()->{
                    //groundIntake.setState(GroundIntake.State.OFF);
                    turret.setState(Turret.State.LEFT);
                    slides.setState(Slides.State.MID_DROP);
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .addDisplacementMarker(13, ()->{
                    intake.setState(Intake.State.OFF);
                })
                .build();
        Trajectory cycleDropOff2 = robot.trajectoryBuilder(initCycle.end())

                .lineToConstantHeading(new Vector2d(-26.95,12.75))
                .addDisplacementMarker(2, ()->{
                    //groundIntake.setState(GroundIntake.State.OFF);
                    turret.setState(Turret.State.LEFT);
                    slides.setState(Slides.State.MID_DROP);
                    fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .addDisplacementMarker(13, ()->{
                    intake.setState(Intake.State.OFF);
                })
                .build();
        Trajectory cycleIntakePrep = robot.trajectoryBuilder(cycleDropOff1.end())
                .lineToConstantHeading(new Vector2d(-45, intakeY))
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

                .lineToConstantHeading(new Vector2d(-63.25,intakeY),
                        robot.getVelocityConstraint(25, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))



                .build();
        Trajectory cycleIntakeLow = robot.trajectoryBuilder(cycleIntakePrep.end())

                .lineToConstantHeading(new Vector2d(-63.5,12.0),robot.getVelocityConstraint(25, 5.939, 13.44),
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
                .lineToConstantHeading(new Vector2d(-13,11.98)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDropOff2.end())
                .addTemporalMarker(0,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })
                .addTemporalMarker(0.1,()->{
                    fourbar.setState(vfourb.State.VERTICAL);
                    //intake.setState(Intake.State.OFF);
                })
                .lineToConstantHeading(new Vector2d(-38,11.98)).build();
        Trajectory endRight = robot.trajectoryBuilder(cycleDropOff1.end())
                .addTemporalMarker(0,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })
                .addTemporalMarker(0.1,()->{
                    fourbar.setState(vfourb.State.VERTICAL);
                    //intake.setState(Intake.State.OFF);
                })
                .lineToConstantHeading(new Vector2d(-55,11.98)).build();
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
            robot.followTrajectory(endMiddle);
            robot.alignUp();
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis()-150 < timer){}
        }
        else if (x==3){
            robot.followTrajectory(cycleIntakePrep);
            robot.followTrajectory(cycleIntakeLow);
            cycleIntake();

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

        while(System.currentTimeMillis()-80< timer){
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
        telemetry.addLine(String.format("Translation Y: %.2f feepackage org.firstinspires.ftc.teamcode.auto;\n" +
                "\n" +
                "import com.acmerobotics.dashboard.FtcDashboard;\n" +
                "import com.acmerobotics.dashboard.telemetry.TelemetryPacket;\n" +
                "import com.acmerobotics.roadrunner.geometry.Pose2d;\n" +
                "import com.acmerobotics.roadrunner.geometry.Vector2d;\n" +
                "import com.acmerobotics.roadrunner.profile.VelocityConstraint;\n" +
                "import com.acmerobotics.roadrunner.trajectory.Trajectory;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                "import com.qualcomm.robotcore.hardware.DcMotor;\n" +
                "import com.qualcomm.robotcore.util.ElapsedTime;\n" +
                "\n" +
                "import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;\n" +
                "import org.firstinspires.ftc.teamcode.Robot;\n" +
                "import org.firstinspires.ftc.teamcode.Slides.Slides;\n" +
                "import org.firstinspires.ftc.teamcode.Transfer.Intake;\n" +
                "import org.firstinspires.ftc.teamcode.Transfer.vfourb;\n" +
                "import org.firstinspires.ftc.teamcode.Turret.Detector;\n" +
                "import org.firstinspires.ftc.teamcode.Turret.Turret;\n" +
                "import org.firstinspires.ftc.teamcode.ground.GroundIntake;\n" +
                "import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;\n" +
                "import org.openftc.apriltag.AprilTagDetection;\n" +
                "import org.openftc.easyopencv.OpenCvCamera;\n" +
                "import org.openftc.easyopencv.OpenCvCameraFactory;\n" +
                "import org.openftc.easyopencv.OpenCvCameraRotation;\n" +
                "import org.openftc.easyopencv.OpenCvWebcam;\n" +
                "\n" +
                "import java.util.ArrayList;\n" +
                "\n" +
                "@Autonomous\n" +
                "public class RightSideAuto extends LinearOpMode {\n" +
                "    ElapsedTime t;\n" +
                "    Robot robot;\n" +
                "    Intake intake;\n" +
                "    Slides slides;\n" +
                "    vfourb fourbar;\n" +
                "    GroundIntake groundIntake;\n" +
                "    Turret turret;\n" +
                "    Detector detector1;\n" +
                "    OpenCvWebcam webcam;\n" +
                "    Pose2d startPose = new Pose2d(-38,61,Math.toRadians(270));\n" +
                "    double timer = 0;\n" +
                "    OpenCvCamera camera;\n" +
                "    AprilTagDetectionPipeline aprilTagDetectionPipeline;\n" +
                "    double intakeY = 12.24;\n" +
                "    static final double FEET_PER_METER = 3.28084;\n" +
                "\n" +
                "    // Lens intrinsics\n" +
                "    // UNITS ARE PIXELS\n" +
                "    // NOTE: this calibration is for the C920 webcam at 800x448.\n" +
                "    // You will need to do your own calibration for other configurations!\n" +
                "    int x = 1;\n" +
                "    double fx = 578.272;\n" +
                "    double fy = 578.272;\n" +
                "    double cx = 402.145;\n" +
                "    double cy = 221.506;\n" +
                "\n" +
                "    // UNITS ARE METERS\n" +
                "    double tagsize = 0.166;\n" +
                "\n" +
                "    AprilTagDetection tagOfInterest = null;\n" +
                "\n" +
                "    @Override\n" +
                "\n" +
                "    public void runOpMode() throws InterruptedException {\n" +
                "        t=new ElapsedTime();\n" +
                "        robot = new Robot(this);\n" +
                "        intake = robot.intake;\n" +
                "        slides = robot.slides;\n" +
                "        fourbar = robot.fourbar;\n" +
                "        groundIntake = robot.groundIntake;\n" +
                "        turret = robot.turret;\n" +
                "        fourbar.setState(vfourb.State.STACK_PRIMED);\n" +
                "        timer = System.currentTimeMillis();\n" +
                "        while(System.currentTimeMillis()-200 < timer){}\n" +
                "        turret.setState(Turret.State.INIT);\n" +
                "\n" +
                "        robot.alignUp();\n" +
                "        //turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);\n" +
                "        //turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);\n" +
                "        //fourbar.setState(vfourb.State.INTAKE_POSITION);\n" +
                "        //camInit();\n" +
                "        /*\n" +
                "        Trajectory preload1 = robot.trajectoryBuilder(startPose)\n" +
                "                .lineToConstantHeading(new Vector2d(-38,40))\n" +
                "                .build();*/\n" +
                "\n" +
                "        //MOVE TO MID JUNCTION, ACTUATE AND DEPOSIT ON MID JUNCTION\n" +
                "        Trajectory preload1 = robot.trajectoryBuilder(startPose)\n" +
                "                .lineToConstantHeading(new Vector2d(-34.3, 21.6),robot.getVelocityConstraint(42.5, 5.939, 13.44),\n" +
                "                        robot.getAccelerationConstraint(50))\n" +
                "                .addDisplacementMarker(1, ()->{\n" +
                "                    fourbar.setState(vfourb.State.ALIGN_POSITION);\n" +
                "                    //robot.aligDownn();\n" +
                "                })\n" +
                "                .addDisplacementMarker(2, ()->{\n" +
                "\n" +
                "                    //groundIntake.setState(GroundIntake.State.INTAKING);\n" +
                "\n" +
                "                    turret.setState(Turret.State.RIGHT);\n" +
                "                    //turret.update();\n" +
                "                    slides.setState(Slides.State.MID_DROP);\n" +
                "\n" +
                "                })\n" +
                "                .addTemporalMarker(2,()->{\n" +
                "                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);\n" +
                "\n" +
                "                })\n" +
                "                .addTemporalMarker(2.1, ()->{\n" +
                "                    intake.setState(Intake.State.DEPOSITING);\n" +
                "                    waitSec(0.25);\n" +
                "                })\n" +
                "                .addDisplacementMarker(()->\n" +
                "                {\n" +
                "                    fourbar.setState(vfourb.State.ALIGN_POSITION);\n" +
                "                    waitSec(0.115);\n" +
                "                })\n" +
                "                .build();\n" +
                "\n" +
                "        //RESET EVERYTHING, MOVE TO STACK, READY FOR PICK UP\n" +
                "        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())\n" +
                "                .addTemporalMarker(0,()->{\n" +
                "                    fourbar.setState(vfourb.State.STACK_PRIMED);\n" +
                "                    slides.setState(Slides.State.BOTTOM);\n" +
                "                    turret.setState(Turret.State.ZERO);\n" +
                "\n" +
                "                    //groundIntake.setState(GroundIntake.State.DEPOSITING);\n" +
                "                    robot.alignUp();\n" +
                "\n" +
                "                })\n" +
                "                .lineToConstantHeading(new Vector2d(-40, 8.0))\n" +
                "                .build();\n" +
                "        Trajectory preload3 = robot.trajectoryBuilder(preload2.end())\n" +
                "\n" +
                "                .lineToConstantHeading(new Vector2d(-40, intakeY))\n" +
                "                .build();\n" +
                "        Trajectory preload4 = robot.trajectoryBuilder(preload3.end())\n" +
                "                /* .addTemporalMarker(0,()->{\n" +
                "                     fourbar.setState(vfourb.State.STACK_PRIMED);\n" +
                "                     slides.setState(Slides.State.BOTTOM);\n" +
                "                     turret.setState(Turret.State.ZERO);\n" +
                "                     groundIntake.setState(GroundIntake.State.DEPOSITING);\n" +
                "                 })*/\n" +
                "                .addTemporalMarker(0.5, ()->{\n" +
                "                    robot.alignDown();\n" +
                "                })\n" +
                "                .lineToLinearHeading(new Pose2d(-41, intakeY, Math.toRadians(180)))\n" +
                "                .build();\n" +
                "        //MOVE TO STACK, PICK UP FIRST CONE\n" +
                "\n" +
                "        Trajectory initCycle = robot.trajectoryBuilder(preload4.end())\n" +
                "                .lineToConstantHeading(new Vector2d(-63.25,intakeY),robot.getVelocityConstraint(25, 5.939, 13.44),\n" +
                "                        robot.getAccelerationConstraint(30))\n" +
                "\n" +
                "\n" +
                "                .addDisplacementMarker(2, ()->{\n" +
                "                    slides.setState(Slides.State.INTAKE_AUTO);\n" +
                "                    //groundIntake.setState(GroundIntake.State.INTAKING);\n" +
                "                    intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "                .build();\n" +
                "\n" +
                "        //MOVE TO MID JUNCTION, ACTUATE AND DROP OFF FIRST CONE\n" +
                "        Trajectory cycleDropOff1 = robot.trajectoryBuilder(initCycle.end())\n" +
                "\n" +
                "                .lineToConstantHeading(new Vector2d(-26.7,12.75))\n" +
                "                .addDisplacementMarker(2, ()->{\n" +
                "                    //groundIntake.setState(GroundIntake.State.OFF);\n" +
                "                    turret.setState(Turret.State.LEFT);\n" +
                "                    slides.setState(Slides.State.MID_DROP);\n" +
                "                    fourbar.setState(vfourb.State.ALIGN_POSITION);\n" +
                "                })\n" +
                "                .addDisplacementMarker(13, ()->{\n" +
                "                    intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "                .build();\n" +
                "        Trajectory cycleDropOff2 = robot.trajectoryBuilder(initCycle.end())\n" +
                "\n" +
                "                .lineToConstantHeading(new Vector2d(-26.95,12.75))\n" +
                "                .addDisplacementMarker(2, ()->{\n" +
                "                    //groundIntake.setState(GroundIntake.State.OFF);\n" +
                "                    turret.setState(Turret.State.LEFT);\n" +
                "                    slides.setState(Slides.State.MID_DROP);\n" +
                "                    fourbar.setState(vfourb.State.ALIGN_POSITION);\n" +
                "                })\n" +
                "                .addDisplacementMarker(13, ()->{\n" +
                "                    intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "                .build();\n" +
                "        Trajectory cycleIntakePrep = robot.trajectoryBuilder(cycleDropOff1.end())\n" +
                "                .lineToConstantHeading(new Vector2d(-45, intakeY))\n" +
                "                .addTemporalMarker(0, ()->{\n" +
                "                    turret.setState(Turret.State.ZERO);\n" +
                "                    slides.setState(Slides.State.BOTTOM);\n" +
                "\n" +
                "                    //groundIntake.setState(GroundIntake.State.DEPOSITING);\n" +
                "                })\n" +
                "\n" +
                "                .build();\n" +
                "        //MOVE TO STACK, PICK UP ANOTHER CONE\n" +
                "        Trajectory cycleIntakeHigh = robot.trajectoryBuilder(cycleIntakePrep.end())\n" +
                "                .addDisplacementMarker(0,()->{\n" +
                "                    robot.slides.setState(Slides.State.INTAKE_AUTO);\n" +
                "                    //groundIntake.setState(GroundIntake.State.INTAKING);\n" +
                "                    intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "\n" +
                "                .lineToConstantHeading(new Vector2d(-63.25,intakeY),\n" +
                "                        robot.getVelocityConstraint(25, 5.939, 13.44),\n" +
                "                        robot.getAccelerationConstraint(30))\n" +
                "\n" +
                "\n" +
                "\n" +
                "                .build();\n" +
                "        Trajectory cycleIntakeLow = robot.trajectoryBuilder(cycleIntakePrep.end())\n" +
                "\n" +
                "                .lineToConstantHeading(new Vector2d(-63.5,12.0),robot.getVelocityConstraint(25, 5.939, 13.44),\n" +
                "                        robot.getAccelerationConstraint(30))\n" +
                "\n" +
                "                .addTemporalMarker(0, ()->{\n" +
                "                    turret.setState(Turret.State.ZERO);\n" +
                "                    slides.setState(Slides.State.BOTTOM);\n" +
                "\n" +
                "                    //groundIntake.setState(GroundIntake.State.DEPOSITING);\n" +
                "                })\n" +
                "                .addDisplacementMarker(15,()->{\n" +
                "                    //groundIntake.setState(GroundIntake.State.INTAKING);\n" +
                "                    intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "                .build();\n" +
                "        Trajectory endLeft = robot.trajectoryBuilder(cycleDropOff2.end())\n" +
                "                .addTemporalMarker(0,()->{\n" +
                "                    turret.setState(Turret.State.ZERO);\n" +
                "                    slides.setState(Slides.State.BOTTOM);\n" +
                "\n" +
                "                })\n" +
                "                .addTemporalMarker(0.1,()->{\n" +
                "                    fourbar.setState(vfourb.State.VERTICAL);\n" +
                "                    //intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "                .lineToConstantHeading(new Vector2d(-13,11.98)).build();\n" +
                "        Trajectory endMiddle = robot.trajectoryBuilder(cycleDropOff2.end())\n" +
                "                .addTemporalMarker(0,()->{\n" +
                "                    turret.setState(Turret.State.ZERO);\n" +
                "                    slides.setState(Slides.State.BOTTOM);\n" +
                "\n" +
                "                })\n" +
                "                .addTemporalMarker(0.1,()->{\n" +
                "                    fourbar.setState(vfourb.State.VERTICAL);\n" +
                "                    //intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "                .lineToConstantHeading(new Vector2d(-38,11.98)).build();\n" +
                "        Trajectory endRight = robot.trajectoryBuilder(cycleDropOff1.end())\n" +
                "                .addTemporalMarker(0,()->{\n" +
                "                    turret.setState(Turret.State.ZERO);\n" +
                "                    slides.setState(Slides.State.BOTTOM);\n" +
                "\n" +
                "                })\n" +
                "                .addTemporalMarker(0.1,()->{\n" +
                "                    fourbar.setState(vfourb.State.VERTICAL);\n" +
                "                    //intake.setState(Intake.State.OFF);\n" +
                "                })\n" +
                "                .lineToConstantHeading(new Vector2d(-55,11.98)).build();\n" +
                "        /*Trajectory cycleIntake = robot.trajectoryBuilder(preload3.end())\n" +
                "                        .lineToConstantHeading(new Vector2d(-48,10))\n" +
                "                                .build();*/\n" +
                "        //robot.thread.run();\n" +
                "        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(\"cameraMonitorViewId\", \"id\", hardwareMap.appContext.getPackageName());\n" +
                "        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, \"Webcam 1\"), cameraMonitorViewId);\n" +
                "        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);\n" +
                "        //FtcDashboard dashboard = FtcDashboard.getInstance();\n" +
                "        camera.setPipeline(aprilTagDetectionPipeline);\n" +
                "        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()\n" +
                "        {\n" +
                "            @Override\n" +
                "            public void onOpened()\n" +
                "            {\n" +
                "                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);\n" +
                "            }\n" +
                "\n" +
                "            @Override\n" +
                "            public void onError(int errorCode)\n" +
                "            {\n" +
                "\n" +
                "            }\n" +
                "        });\n" +
                "        //dashboard.startCameraStream(webcam, 30);\n" +
                "        telemetry.setMsTransmissionInterval(50);\n" +
                "\n" +
                "        /*\n" +
                "         * The INIT-loop:\n" +
                "         * This REPLACES waitForStart!\n" +
                "         */\n" +
                "        TelemetryPacket packet =new TelemetryPacket();\n" +
                "        while (!isStarted() && !isStopRequested())\n" +
                "        {\n" +
                "            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();\n" +
                "            telemetry.addLine(String.format(\"detections\", currentDetections.size()));\n" +
                "            if(currentDetections.size()>0) {\n" +
                "                tagToTelemetry(currentDetections.get(0));\n" +
                "                tagOfInterest = currentDetections.get(0);\n" +
                "            }\n" +
                "            //dashboard.sendTelemetryPacket(packet);\n" +
                "            telemetry.update();\n" +
                "            sleep(20);\n" +
                "        }\n" +
                "\n" +
                "        if(tagOfInterest != null)\n" +
                "        {\n" +
                "\n" +
                "            telemetry.addLine(\"Tag snapshot:\\n\");\n" +
                "            tagToTelemetry(tagOfInterest);\n" +
                "            telemetry.update();\n" +
                "        }\n" +
                "        else\n" +
                "        {\n" +
                "            telemetry.addLine(\"No tag snapshot available, it was never sighted during the init loop :(\");\n" +
                "            telemetry.update();\n" +
                "        }\n" +
                "\n" +
                "        /* Actually do something useful */\n" +
                "        if(tagOfInterest == null)\n" +
                "        {\n" +
                "            /*\n" +
                "             * Insert your autonomous code here, presumably running some default configuration\n" +
                "             * since the tag was never sighted during INIT\n" +
                "             */\n" +
                "        }\n" +
                "        else\n" +
                "        {\n" +
                "\n" +
                "            if(tagOfInterest.id == 4)\n" +
                "            {\n" +
                "                x =1;\n" +
                "            }\n" +
                "            else if(tagOfInterest.id == 7)\n" +
                "            {\n" +
                "                x=2;\n" +
                "            }\n" +
                "            else if(tagOfInterest.id == 8)\n" +
                "            {\n" +
                "                x=3;\n" +
                "            }\n" +
                "        }\n" +
                "        if (isStopRequested()) return;\n" +
                "        timer = System.currentTimeMillis();\n" +
                "        //preload\n" +
                "        robot.setPoseEstimate(startPose);\n" +
                "        robot.followTrajectory(preload1);\n" +
                "        robot.followTrajectory(preload2);\n" +
                "        robot.followTrajectory(preload3);\n" +
                "        robot.followTrajectory(preload4);\n" +
                "        groundIntake.setState(GroundIntake.State.OFF);\n" +
                "        //robot.turn(Math.toRadians(-100));\n" +
                "        //1st cycle\n" +
                "        robot.followTrajectory(initCycle);\n" +
                "        cycleIntake();\n" +
                "        robot.followTrajectory(cycleDropOff1);\n" +
                "        cycleDeposit();\n" +
                "        //2nd cycle\n" +
                "        robot.followTrajectory(cycleIntakePrep);\n" +
                "        robot.followTrajectory(cycleIntakeHigh);\n" +
                "        cycleIntake();\n" +
                "        robot.followTrajectory(cycleDropOff1);\n" +
                "        cycleDeposit();\n" +
                "        //3rd cycle\n" +
                "        robot.followTrajectory(cycleIntakePrep);\n" +
                "        robot.followTrajectory(cycleIntakeLow);\n" +
                "        cycleIntake();\n" +
                "        robot.followTrajectory(cycleDropOff2);\n" +
                "        cycleDeposit();\n" +
                "        //picking up 4th cone?\n" +
                "        /*\n" +
                "        robot.followTrajectory(cycleIntakePrep);\n" +
                "        robot.followTrajectory(cycleIntakeLow);\n" +
                "        cycleIntake();\n" +
                "        robot.followTrajectory(cycleDropOff1);\n" +
                "        cycleDeposit();*/\n" +
                "        //park\n" +
                "        if(x == 1){\n" +
                "            robot.followTrajectory(endLeft);\n" +
                "            robot.alignUp();\n" +
                "            timer = System.currentTimeMillis();\n" +
                "            while(System.currentTimeMillis()-150 < timer){}\n" +
                "        }\n" +
                "        else if( x==2){\n" +
                "            robot.followTrajectory(endMiddle);\n" +
                "            robot.alignUp();\n" +
                "            timer = System.currentTimeMillis();\n" +
                "            while(System.currentTimeMillis()-150 < timer){}\n" +
                "        }\n" +
                "        else if (x==3){\n" +
                "            robot.followTrajectory(cycleIntakePrep);\n" +
                "            robot.followTrajectory(cycleIntakeLow);\n" +
                "            cycleIntake();\n" +
                "\n" +
                "        }\n" +
                "        //4th cycle\n" +
                "        /*\n" +
                "        robot.followTrajectory(cycleIntakeLow);\n" +
                "        cycleIntake();\n" +
                "        robot.followTrajectory(cycleDropOff1);\n" +
                "        cycleDeposit();\n" +
                "        robot.followTrajectory(cycleIntakeLow);*/\n" +
                "    }\n" +
                "    //if(isStopRequested()) return;\n" +
                "\n" +
                "\n" +
                "    private void cycleIntake(){\n" +
                "\n" +
                "        //timer = System.currentTimeMillis();\n" +
                "        robot.intake.setState(Intake.State.INTAKING);\n" +
                "        robot.slides.setState(Slides.State.BOTTOM);\n" +
                "        //while(System.currentTimeMillis()- < timer){}\n" +
                "        //robot.groundIntake.setState(GroundIntake.State.DEPOSITING);\n" +
                "        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);\n" +
                "\n" +
                "        timer = System.currentTimeMillis();\n" +
                "        while(System.currentTimeMillis()-400 < timer){}\n" +
                "        //robot.intake.setState(Intake.State.OFF);\n" +
                "        fourbar.setState(vfourb.State.DEPOSIT_POSITION);\n" +
                "        timer = System.currentTimeMillis();\n" +
                "        while(System.currentTimeMillis()-40 < timer){}\n" +
                "    }\n" +
                "    private void cycleDeposit(){\n" +
                "        robot.fourbar.setState(vfourb.State.DEPOSIT_POSITION);\n" +
                "        timer = System.currentTimeMillis();\n" +
                "\n" +
                "        while(System.currentTimeMillis()-80< timer){\n" +
                "            //turret.autoAlign();\n" +
                "        }\n" +
                "        intake.setState(Intake.State.DEPOSITING);\n" +
                "        timer = System.currentTimeMillis();\n" +
                "\n" +
                "        while(System.currentTimeMillis()-300< timer){\n" +
                "            //turret.autoAlign();\n" +
                "        }\n" +
                "        robot.fourbar.setState(vfourb.State.STACK_PRIMED);\n" +
                "    }\n" +
                "    /*\n" +
                "    public void camInit() {\n" +
                "        final int CAMERA_WIDTH = 320; // width  of wanted camera resolution\n" +
                "        final int CAMERA_HEIGHT = 240; // height of wanted camera resolution\n" +
                "        int cameraMonitorViewId = this\n" +
                "                .hardwareMap\n" +
                "                .appContext\n" +
                "                .getResources().getIdentifier(\n" +
                "                        \"cameraMonitorViewId\",\n" +
                "                        \"id\",\n" +
                "                        hardwareMap.appContext.getPackageName()\n" +
                "                );\n" +
                "        webcam = OpenCvCameraFactory\n" +
                "                .getInstance()\n" +
                "                .createWebcam(hardwareMap.get(WebcamName.class, \"Webcam 1\"), cameraMonitorViewId);\n" +
                "        webcam.setPipeline(detector1 = new Detector());\n" +
                "        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {\n" +
                "            @Override\n" +
                "            public void onOpened() {\n" +
                "                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);\n" +
                "                System.out.println(\"START\");\n" +
                "            }\n" +
                "            public void onError(int errorCode) {\n" +
                "            }\n" +
                "        });\n" +
                "        //dashboard.startCameraStream(webcam, 30);\n" +
                "        telemetry.addLine(\"waiting for start\");\n" +
                "        telemetry.update();\n" +
                "    }*/\n" +
                "    public void waitSec(double seconds)\n" +
                "    {\n" +
                "        t.reset();\n" +
                "        while(t.milliseconds()<seconds*1000)\n" +
                "        {\n" +
                "            //stall\n" +
                "        }\n" +
                "    }\n" +
                "    void tagToTelemetry(AprilTagDetection detection)\n" +
                "    {\n" +
                "        telemetry.addLine(String.format(\"\\nDetected tag ID=%d\", detection.id));\n" +
                "        telemetry.addLine(String.format(\"Translation X: %.2f feet\", detection.pose.x*FEET_PER_METER));\n" +
                "        telemetry.addLine(String.format(\"Translation Y: %.2f feet\", detection.pose.y*FEET_PER_METER));\n" +
                "        telemetry.addLine(String.format(\"Translation Z: %.2f feet\", detection.pose.z*FEET_PER_METER));\n" +
                "        telemetry.addLine(String.format(\"Rotation Yaw: %.2f degrees\", Math.toDegrees(detection.pose.yaw)));\n" +
                "        telemetry.addLine(String.format(\"Rotation Pitch: %.2f degrees\", Math.toDegrees(detection.pose.pitch)));\n" +
                "        telemetry.addLine(String.format(\"Rotation Roll: %.2f degrees\", Math.toDegrees(detection.pose.roll)));\n" +
                "    }\n" +
                "}\nt", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
