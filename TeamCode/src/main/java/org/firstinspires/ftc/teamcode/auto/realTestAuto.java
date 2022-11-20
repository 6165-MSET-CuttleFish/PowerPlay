package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.Turret.Turret2;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class realTestAuto extends LinearOpMode
{
    ElapsedTime timer;
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret2 turret;
    Detector detector1;
    OpenCvWebcam webcam;
    Pose2d startPose = new Pose2d(0,0);

    public enum State
    {
        GET, PLACE, INTAKE, DEPOSIT
    }
    State state;

    @Override
    public void runOpMode() throws InterruptedException {
        timer=new ElapsedTime();
        robot = new Robot(hardwareMap);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret=new Turret2(hardwareMap);
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourbar.setState(vfourb.State.PRIMED);

        Trajectory initial = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(0, 51))
                .build();

        Trajectory intake=robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-10, 51))

                .build();

        Trajectory deposit=robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(0, 51))
                .build();


        waitForStart();
        if(isStopRequested()) return;

        robot.setPoseEstimate(startPose);
        robot.followTrajectoryAsync(initial);
        prepDeposit();
        while(robot.isBusy()&&turret.isBusy())
        {
            robot.update();
            turret.update();
        }
        //solve the eternal problem
        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
        timer.reset();
        while(timer.milliseconds()<1500)
        {
            //stall
        }
        this.intake.setState(Intake.State.DEPOSITING);
        while(timer.milliseconds()<3000)
        {

        }

        /*state=State.GET;
        prepIntake();
        robot.followTrajectoryAsync(intake);

        while(opModeIsActive())
        {
            switch(state)
            {
                case GET:
                    if(!robot.isBusy()&&!turret.isBusy())
                    {
                        intake();
                        timer.reset();
                        state=state.INTAKE;
                    }
                    break;
                case INTAKE:
                    if(timer.milliseconds()>2500)
                    {
                        prepDeposit();
                        robot.followTrajectoryAsync(deposit);
                        state=State.PLACE;
                    }
                    break;
                case PLACE:



            }
            robot.update();
            turret.update();
        }*/
    }

    public void prepIntake()
    {
        slides.setState(Slides.State.BOTTOM);
        fourbar.setState(vfourb.State.PRIMED);
        turret.setState(Turret2.State.ZERO);
    }
    public void prepDeposit()
    {
        slides.setState(Slides.State.HIGH);
        fourbar.setState(vfourb.State.ALIGN_POSITION);
        turret.setState(Turret2.State.MIDRIGHT);
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
