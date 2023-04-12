package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.util.BackgroundTasks;
@TeleOp
public class TeleMacro extends LinearOpMode {
    Robot robot;
    Slides slides;
    Deposit deposit;
    GroundIntake groundIntake;
    Claw claw;
    Turret turret;
    BackgroundTasks hardware;
    TelemetryPacket packet;
    GamepadEx primary, secondary;
    ToggleButtonReader macroToggle, angling;
    ButtonReader cycle;
    KeyReader[] keyReaders;
    double timer;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        slides = robot.slides;
        claw = robot.claw;
        deposit = robot.deposit;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setState(Slides.State.BOTTOM);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        //claw.setState(Claw.State.CLOSE);
        turret.setState(Turret.State.ZERO);

        Trajectory cycleDrop = robot.trajectoryBuilder(new Pose2d(4.5,0,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(0, 0),robot.getVelocityConstraint(25, 5.939, 13.44),
                        robot.getAccelerationConstraint(60))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.CYCLE_HIGH);
                })
                .addTemporalMarker(0.2, ()->{
                    turret.setState(Turret.State.BACK);
                })
                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(4.5, 0),robot.getVelocityConstraint(25, 5.939, 13.44),
                        robot.getAccelerationConstraint(60))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.BOTTOM);
                })
                .addTemporalMarker(0.2, ()->{
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);

                })
                .build();
        keyReaders = new KeyReader[]{
                macroToggle = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                angling = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                cycle = new ButtonReader(primary, GamepadKeys.Button.B)
        };
        waitForStart();
        while(opModeIsActive()){
            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }
            if(macroToggle.getState()){
                robot.setPoseEstimate(new Pose2d(0,0, Math.toRadians(0)));
                if(cycle.wasJustPressed()){
                    robot.followTrajectory(cycleIntake);
                    intake();
                    robot.followTrajectory(cycleDrop);
                    dropOff();
                }
            }
            else{
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * 0.4,
                                gamepad1.left_stick_x * 0.4,
                                -gamepad1.right_stick_x * 0.4
                        )
                );
                if(angling.getState()){
                    slides.setState(Slides.State.HIGH);
                    timer = System.currentTimeMillis();
                    while(System.currentTimeMillis()-350 < timer){
                        robot.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.4,
                                        gamepad1.left_stick_x * 0.4,
                                        -gamepad1.right_stick_x * 0.4
                                )
                        );
                        robot.update();
                    }
                    turret.setState(Turret.State.BACK);
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                }
                else{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                    deposit.setAngle(Deposit.AngleState.INTAKE);
                    timer = System.currentTimeMillis();
                    while(System.currentTimeMillis()-350 < timer){
                        robot.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.4,
                                        gamepad1.left_stick_x * 045,
                                        -gamepad1.right_stick_x * 0.4
                                )
                        );
                        robot.update();
                    }
                }
            }
        }
    }
    public void dropOff(){
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-500 < timer){

            robot.update();
        }
        deposit.setExtension(Deposit.ExtensionState.EXTEND);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-350 < timer){

            robot.update();
        }
        claw.setState(Claw.State.OPEN);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-95< timer){
            robot.update();
        }
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        turret.setState(Turret.State.ZERO);
        slides.setState(Slides.State.BOTTOM);

    }
    public void intake(){
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-75< timer){
            robot.update();
        }
        claw.setState(Claw.State.CLOSE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-230< timer){
            robot.update();
        }
        slides.setState(Slides.State.HIGH);

        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-200< timer){
            robot.update();
        }
        deposit.setAngle(Deposit.AngleState.VECTORING);

        deposit.setExtension(Deposit.ExtensionState.RETRACT);
    }
}
