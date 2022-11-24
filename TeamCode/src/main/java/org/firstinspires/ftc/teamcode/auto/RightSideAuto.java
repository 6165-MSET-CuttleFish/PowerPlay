package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RightSideAuto extends LinearOpMode {
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
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourbar.setState(vfourb.State.INTAKE_POSITION);
        //camInit();
        /*
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-38,40))
                .build();*/

        //MOVE TO MID JUNCTION, ACTUATE AND DEPOSIT ON MID JUNCTION
        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-34.5, 20.88))
                        .addDisplacementMarker(2, ()->{
                            groundIntake.setState(GroundIntake.State.DEPOSITING);
                            turret.setState(Turret.State.RIGHT);
                            slides.setState(Slides.State.MID_DROP);
                            fourbar.setState(vfourb.State.ALIGN_POSITION);
                        })
                .addTemporalMarker(2,()->{
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);

                })
                .addTemporalMarker(2.1, ()->{
                    intake.setState(Intake.State.DEPOSITING);
                })
                .build();

        //RESET EVERYTHING, MOVE TO STACK, READY FOR PICK UP
        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .addTemporalMarker(0,()->{
                    fourbar.setState(vfourb.State.STACK_PRIMED);
                    slides.setState(Slides.State.BOTTOM);
                    turret.setState(Turret.State.ZERO);
                    groundIntake.setState(GroundIntake.State.DEPOSITING);
                })
                .lineToLinearHeading(new Pose2d(-40, 11.76, Math.toRadians(178)))
                .build();

        //MOVE TO STACK, PICK UP FIRST CONE
        Trajectory initCycle = robot.trajectoryBuilder(preload2.end())
                .lineToConstantHeading(new Vector2d(-63,11.76))
                .addDisplacementMarker(2, ()->{
                    slides.setState(Slides.State.INTAKE_AUTO);
                    groundIntake.setState(GroundIntake.State.INTAKING);
                    intake.setState(Intake.State.OFF);
                })
                .build();

        //MOVE TO MID JUNCTION, ACTUATE AND DROP OFF FIRST CONE
        Trajectory cycleDropOff1 = robot.trajectoryBuilder(initCycle.end())
                .lineToConstantHeading(new Vector2d(-25.75,13))
                .addDisplacementMarker(2, ()->{

                    turret.setState(Turret.State.LEFT);
                    slides.setState(Slides.State.MID_DROP);
                    //fourbar.setState(vfourb.State.ALIGN_POSITION);
                })
                .build();

        //MOVE TO STACK, PICK UP ANOTHER CONE
        Trajectory cycleIntakeHigh = robot.trajectoryBuilder(cycleDropOff1.end())

                .lineToConstantHeading(new Vector2d(-63,11.76),
                        robot.getVelocityConstraint(45, 5.939, 14.48),
                        robot.getAccelerationConstraint(25))

                .addTemporalMarker(0, ()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);
                    intake.setState(Intake.State.OFF);
                    groundIntake.setState(GroundIntake.State.INTAKING);
                })
                .addDisplacementMarker(15,()->{
                    robot.slides.setState(Slides.State.INTAKE_AUTO);
                })
                .build();
        Trajectory cycleIntakeLow = robot.trajectoryBuilder(cycleDropOff1.end())

                .lineToConstantHeading(new Vector2d(-63,11.76),
                        robot.getVelocityConstraint(45, 5.939, 14.48),
                        robot.getAccelerationConstraint(25))

                .addTemporalMarker(0, ()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);
                    intake.setState(Intake.State.OFF);
                    groundIntake.setState(GroundIntake.State.INTAKING);
                })
                .build();
        Trajectory endLeft = robot.trajectoryBuilder(cycleDropOff1.end())
                .addTemporalMarker(0,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })
                .addTemporalMarker(0.1,()->{
                    fourbar.setState(vfourb.State.VERTICAL);
                    intake.setState(Intake.State.OFF);
                })
                .lineToConstantHeading(new Vector2d(-13,11.98)).build();
        Trajectory endMiddle = robot.trajectoryBuilder(cycleDropOff1.end())
                .addTemporalMarker(0,()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })
                .addTemporalMarker(0.1,()->{
                    fourbar.setState(vfourb.State.VERTICAL);
                    intake.setState(Intake.State.OFF);
                })
                .lineToConstantHeading(new Vector2d(-33,11.98)).build();
        /*Trajectory cycleIntake = robot.trajectoryBuilder(preload3.end())
                        .lineToConstantHeading(new Vector2d(-48,10))
                                .build();*/

        waitForStart();
        if(opModeIsActive()){
            timer = System.currentTimeMillis();
            //preload
            robot.setPoseEstimate(startPose);
            robot.followTrajectory(preload1);
            robot.followTrajectory(preload2);
            //robot.turn(Math.toRadians(-100));
            //1st cycle
            robot.followTrajectory(initCycle);
            cycleIntake();
            robot.followTrajectory(cycleDropOff1);
            cycleDeposit();

        robot.followTrajectory(cycleIntakeHigh);
        cycleIntake();

        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();

        //3rd cycle
        robot.followTrajectory(cycleIntakeHigh);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();

        //4th cycle
        robot.followTrajectory(cycleIntakeLow);
        cycleIntake();
        robot.followTrajectory(cycleDropOff1);
        cycleDeposit();
        robot.followTrajectory(endMiddle);
        }
        //if(isStopRequested()) return;

    }
    private void cycleIntake(){
        robot.fourbar.setState(vfourb.State.INTAKE_POSITION);
        robot.intake.setState(Intake.State.INTAKING);
        robot.slides.setState(Slides.State.BOTTOM);
        robot.groundIntake.setState(GroundIntake.State.OFF);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-600 < timer){}
        robot.intake.setState(Intake.State.OFF);
        fourbar.setState(vfourb.State.ALIGN_POSITION);
    }
    private void cycleDeposit(){
        robot.fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        timer = System.currentTimeMillis();

        while(System.currentTimeMillis()-50< timer){
            //turret.autoAlign();
        }
        intake.setState(Intake.State.DEPOSITING);
        timer = System.currentTimeMillis();

        while(System.currentTimeMillis()-100< timer){
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
}
