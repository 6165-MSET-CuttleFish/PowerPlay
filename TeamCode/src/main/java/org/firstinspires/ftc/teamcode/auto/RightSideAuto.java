package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
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
    Pose2d startPose = new Pose2d(-61,-32,Math.toRadians(180));

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
        Trajectory preload1 = robot.trajectoryBuilder(new Pose2d(-61,-32, Math.toRadians(270)))

                .lineToConstantHeading(new Vector2d(-59, -12))

                .build();

        Trajectory preload2 = robot.trajectoryBuilder(preload1.end())
                .lineToConstantHeading(new Vector2d(-25, -12))
                .addDisplacementMarker(2, ()->{
                            slides.setState(Slides.State.HIGH);
                            fourbar.setState(vfourb.State.ALIGN_POSITION);
                        }
                )
                .build();
        Trajectory preload3 = robot.trajectoryBuilder(preload2.end())

                .lineToConstantHeading(new Vector2d(-25,-8), robot.getVelocityConstraint(10, 5.939, 14.48),
                        robot.getAccelerationConstraint(45))
                .addTemporalMarker(0.25,()->{
                    //fourbar.setState(vfourb.State.DEPOSIT_POSITION);

                })
                .addTemporalMarker(2.5, ()->{
                    intake.setState(Intake.State.DEPOSITING);
                })

                .build();
        Trajectory preload4 = robot.trajectoryBuilder(preload3.end())
                .lineToConstantHeading(new Vector2d(-22,48))
                .addDisplacementMarker(2,()->{
                    fourbar.setState(vfourb.State.PRIMED);
                    intake.setState(Intake.State.OFF);
                    slides.setState(Slides.State.BOTTOM);
                })
                .build();



        waitForStart();
        if(isStopRequested()) return;

        robot.setPoseEstimate(startPose);
        robot.turn(Math.toRadians(100));
        robot.updatePoseEstimate();
        robot.followTrajectory(preload1);
        robot.followTrajectory(preload2);
        //robot.followTrajectory(preload3);
        //robot.followTrajectory(preload4);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
