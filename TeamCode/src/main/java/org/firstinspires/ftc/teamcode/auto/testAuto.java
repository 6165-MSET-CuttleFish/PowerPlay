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
public class testAuto extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(0,0);



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
        fourbar.setState(vfourb.State.PRIMED);

        Trajectory initial = robot.trajectoryBuilder(robot.getPoseEstimate())

                .lineToConstantHeading(new Vector2d(0, 51))
                .addDisplacementMarker(35, ()->{
                           prepDeposit();
                        }
                )
                .build();

        Trajectory intake=robot.trajectoryBuilder(robot.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-10, 51))
                .addDisplacementMarker(1, ()->{
                            prepIntake();
                        }
                )
                                .build();

        Trajectory deposit=robot.trajectoryBuilder(robot.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(0, 51))
                .addDisplacementMarker(1, ()->{
                           prepDeposit();
                        }
                )
                                .build();


        waitForStart();
        if(isStopRequested()) return;

        robot.setPoseEstimate(startPose);
        robot.followTrajectory(initial);
        deposit();

        while (!isStopRequested() && opModeIsActive())
        {
            robot.followTrajectory(intake);
            intake();
            robot.followTrajectory(deposit);
            deposit();
        }
    }

    public void prepIntake()
    {
        slides.setState(Slides.State.BOTTOM);
        fourbar.setState(vfourb.State.PRIMED);
        turret.setState(Turret.State.ZERO);
    }
    public void prepDeposit()
    {
        slides.setState(Slides.State.HIGH);
        fourbar.setState(vfourb.State.ALIGN_POSITION);
        //turret.setState(Turret.State.RIGHT_MIDDLE);
    }
    public void intake()
    {
        fourbar.setState(vfourb.State.INTAKE_POSITION);
        intake.setState(Intake.State.INTAKING);
    }
    public void deposit()
    {
        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        intake.setState(Intake.State.DEPOSITING);
    }
}
