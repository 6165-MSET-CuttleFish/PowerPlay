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
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.transfer.vfourb;
import org.firstinspires.ftc.teamcode.modules.turret.Detector;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class RebuildRightSide extends LinearOpMode {
    ElapsedTime t;
    Robot robot;
    Slides slides;
    Claw claw;
    Deposit deposit;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(270));
    double timer = 0;
    int cycle = 0;
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
        t = new ElapsedTime();
        robot = new Robot(this);
        deposit = robot.deposit;
        claw = robot.claw;
        slides = robot.slides;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        slides.setState(Slides.State.BOTTOM);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        claw.setState(Claw.State.OPEN);
        turret.setState(Turret.State.ZERO);
        timer = System.currentTimeMillis();
        while (System.currentTimeMillis() - 200 < timer) {
        }

        //MOVE TO MID JUNCTION, ACTUATE AND DEPOSIT ON MID JUNCTION
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35,15))
                .addTemporalMarker(0, () -> {
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                    slides.setState(Slides.State.MID);
                })
                .addTemporalMarker(0.3, () -> {
                    turret.setState(Turret.State.BACK);
                })
                .addTemporalMarker(0.6, () -> {
                   // deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    slides.setState(Slides.State.HIGH);
                })
                .build();

        //RESET EVERYTHING, MOVE TO STACK, READY FOR PICK UP
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .lineToLinearHeading(new Pose2d(-31,7,Math.toRadians(135)))
                .build();
        //MOVE TO MID JUNCTION, ACTUATE AND DROP OFF FIRST CONE
        Trajectory goToStack = robot.trajectoryBuilder(preload2.end())
                .splineTo(new Vector2d(-60, 12),Math.toRadians(180))
                .addTemporalMarker(0, () -> {
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    turret.setState(Turret.State.ZERO);
                })
                .addTemporalMarker(0.5, () -> {
                    if(cycle == 0){
                        slides.setState(Slides.State.CYCLE0);
                    }
                    else if(cycle ==1){
                        slides.setState(Slides.State.CYCLE1);
                    }
                    else if(cycle ==2){
                        slides.setState(Slides.State.CYCLE2);
                    }
                    else if(cycle ==3){
                        slides.setState(Slides.State.CYCLE3);
                    }
                    else if(cycle ==4){
                        slides.setState(Slides.State.CYCLE4);
                    }
                })

                .build();
        Trajectory cycleDropOff = robot.trajectoryBuilder(goToStack.end(), true)
                .splineTo(new Vector2d(-31, 7),Math.toRadians(315))
                .addTemporalMarker(0, () -> {
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                    slides.setState(Slides.State.MID);
                })
                .addTemporalMarker(0.3, () -> {
                    turret.setState(Turret.State.BACK);
                })
                .addTemporalMarker(0.6, () -> {
                    // deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    slides.setState(Slides.State.HIGH);
                })
                .build();
        Trajectory endLeft = robot.trajectoryBuilder(cycleDropOff.end())
                .addTemporalMarker(0, () -> {
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    turret.setState(Turret.State.ZERO);
                })
                .addTemporalMarker(0.5, () -> {

                    slides.setState(Slides.State.BOTTOM);

                })

                .lineToConstantHeading(new Vector2d(-13, 11.98)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDropOff.end())
                .addTemporalMarker(0, () -> {
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    turret.setState(Turret.State.ZERO);
                })
                .addTemporalMarker(0.5, () -> {

                    slides.setState(Slides.State.BOTTOM);
                })

                .lineToConstantHeading(new Vector2d(-40, 11.98)).build();
        Trajectory endRight = robot.trajectoryBuilder(cycleDropOff.end())
                .addTemporalMarker(0, () -> {
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    turret.setState(Turret.State.ZERO);
                })
                .addTemporalMarker(0.5, () -> {

                    slides.setState(Slides.State.BOTTOM);
                })

                .lineToConstantHeading(new Vector2d(-60, 11.98)).build();
        /*Trajectory cycleIntake = robot.trajectoryBuilder(preload3.end())
                        .lineToConstantHeading(new Vector2d(-48,10))
                                .build();*/
        //robot.thread.run();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        //dashboard.startCameraStream(webcam, 30);
        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        TelemetryPacket packet = new TelemetryPacket();
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            telemetry.addLine(String.format("detections", currentDetections.size()));
            if (currentDetections.size() > 0) {
                tagToTelemetry(currentDetections.get(0));
                tagOfInterest = currentDetections.get(0);
            }
            //dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest != null) {

            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        } else {

            if (tagOfInterest.id == 4) {
                x = 1;
            } else if (tagOfInterest.id == 7) {
                x = 2;
            } else if (tagOfInterest.id == 8) {
                x = 3;
            }
        }
        if (isStopRequested()) return;
        timer = System.currentTimeMillis();
        //preload
        robot.setPoseEstimate(startPose);
        claw.setState(Claw.State.CLOSE);
        wait(500);

        robot.followTrajectory(preload1);
        robot.followTrajectory(preload2);
        claw.setState(Claw.State.OPEN);
        wait(500);

        robot.followTrajectory(goToStack);
        cycle++;
        claw.setState(Claw.State.CLOSE);
        wait(500);

        robot.followTrajectory(cycleDropOff);
        claw.setState(Claw.State.OPEN);
        wait(500);

        //park

/*        if (x == 1) {
            robot.followTrajectory(endLeft);
//            robot.alignUp();
            timer = System.currentTimeMillis();
            while (System.currentTimeMillis() - 150 < timer) {
            }
            //cycleIntake();
        } else if (x == 2) {
            robot.followTrajectory(endMiddle);
//            robot.alignUp();
            timer = System.currentTimeMillis();
            while (System.currentTimeMillis() - 150 < timer) {
            }
        } else if (x == 3) {
            robot.followTrajectory(endRight);

        }*/
        //4th cycle
        /*
        robot.followTrajectory(cycleIntakeLow);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();
        robot.followTrajectory(cycleIntakeLow);*/
    }
    //if(isStopRequested()) return;

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
