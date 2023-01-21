package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.openftc.easyopencv.OpenCvWebcam;

public class RightSideMS extends LinearOpMode {
    ElapsedTime t;
    RobotTemp robot;
    Intake intake;
    Slides slides;
    Claw claw;
    Deposit deposit;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(-38,61,Math.toRadians(270));
    double timer = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotTemp(this, true);
        intake = robot.intake;
        slides = robot.slides;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        claw = robot.claw;
        deposit = robot.deposit;
        slides.setState(Slides.State.BOTTOM);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        claw.setState(Claw.State.OPEN);
        turret.setState(Turret.State.ZERO);

        Trajectory preload1 = robot.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-38, 51, Math.toRadians(180)))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.HIGH);

                })
                .build();
        
        waitForStart();
        robot.setPoseEstimate(startPose);
        robot.followTrajectory(preload1);
    }
}
