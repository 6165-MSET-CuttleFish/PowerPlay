package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotTemp;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Detector;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.transfer.vfourb;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.BackgroundCR;

@TeleOp
@Config
public class ASafeDriverControl extends LinearOpMode {
    RobotTemp robot;
    Slides slides;
    Deposit deposit;
    GroundIntake groundIntake;
    Claw claw;
    Turret turret;
    BackgroundCR hardware;
    TelemetryPacket packet;
    GamepadEx primary, secondary;
    boolean transfer = false;
    boolean resetCheck, cycleCheck = false;
    int turretPos = -1; //0 = left, 1 = back, 2 = right
    double ninjaMultiplier = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    KeyReader[] keyReaders;
    TriggerReader intakeTransfer, depositTransfer, actuateUp;
    ButtonReader autoAlign, cancelMacro, cycleMacro, cycleDown, cycleUp, actuateLeft,
            intakeGround, extakeGround, actuateRight, reset, half, angle, extension, turretZero, conePickup;
    ToggleButtonReader ninjaMode, straightMode;

    Gamepad.RumbleEffect customRumbleEffect0;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect2;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect3;    // Use to build a custom rumble sequence.

    int cycleValue = 0;
    boolean slidesZero = false, turretStop = false;
    boolean autoActuate = false;
    public double resetTime = 1;
    public double transferTime = 1, transferTurretTime = 500;
    public boolean distanceSensor = false;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime resetTimer = new ElapsedTime();
    ElapsedTime cycleTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotTemp(this);
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

        keyReaders = new KeyReader[]{
                ninjaMode = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                intakeGround = new ToggleButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                extakeGround = new ToggleButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                straightMode = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                turretZero = new ToggleButtonReader(primary, GamepadKeys.Button.X),
                cycleMacro = new ButtonReader(primary, GamepadKeys.Button.B),
                cancelMacro = new ButtonReader(primary, GamepadKeys.Button.A),
                autoAlign = new ButtonReader(primary, GamepadKeys.Button.Y),
                intakeTransfer = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),

                actuateRight = new ButtonReader(secondary, GamepadKeys.Button.DPAD_RIGHT),
                actuateLeft = new ButtonReader(secondary, GamepadKeys.Button.DPAD_LEFT),
                actuateUp = new TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                depositTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.LEFT_TRIGGER),
                reset = new ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN),
                half = new ButtonReader(secondary, GamepadKeys.Button.B),
                angle = new ButtonReader(secondary, GamepadKeys.Button.Y),
                conePickup = new ButtonReader(secondary, GamepadKeys.Button.X),
                extension = new ButtonReader(secondary, GamepadKeys.Button.A),
                cycleDown = new ButtonReader(secondary, GamepadKeys.Button.LEFT_BUMPER),
                cycleUp = new ButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER),

        };
        customRumbleEffect0 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 200)
                .addStep(0.0, 0.0, 1000) //  Rumble right motor 100% for 500 mSec
                .build();
        customRumbleEffect1 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1000)  //  Rumble right motor 100% for 500 mSec
                .build();
        customRumbleEffect2 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1000) //  Rumble right motor 100% for 500 mSec
                .build();
        customRumbleEffect3 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1000) //  Rumble right motor 100% for 500 mSec
                .build();
        Trajectory cycleIntake = robot.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .forward(10, RobotTemp.getVelocityConstraint(10, 5.939, 14.48), RobotTemp.getAccelerationConstraint(45))
                .addTemporalMarker(0, this::resetUpdate)
                .addTemporalMarker(0, ()->{
                    groundIntake.setState(GroundIntake.State.INTAKING);
                })
                .build();

        Trajectory cycleDropOff = robot.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .back(10, RobotTemp.getVelocityConstraint(10, 5.939, 14.48), RobotTemp.getAccelerationConstraint(45))
                .addTemporalMarker(0,()-> {
                    slides.setState(Slides.State.HIGH);
                })
                .addTemporalMarker(1, () -> {
                    turret.setState(Turret.State.BACK);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                })
                .build();

        waitForStart();
        gamepad1.setLedColor(9, 79, 183, 120000); //blue
        gamepad2.setLedColor(147, 112, 219, 120000); //light purple
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        claw.setState(Claw.State.OPEN);
        robot.midOdo.setPosition(0);
        robot.sideOdo.setPosition(0.65);
        turret.setState(Turret.State.ZERO);
        while (opModeIsActive()) {
            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }
            if (ninjaMode.wasJustPressed() && ninjaMultiplier == 1) {
                ninjaMultiplier = 0.6;
            } else if (ninjaMode.wasJustPressed() && ninjaMultiplier == 0.6) {
                ninjaMultiplier = 1;
            }

            //DRIVETRAIN
            if (straightMode.getState()) {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * ninjaMultiplier,
                                0,
                                -gamepad1.right_stick_x * ninjaMultiplier
                        )
                );
            } else robot.setWeightedDrivePower(
                    new Pose2d(
                            Math.abs(gamepad1.left_stick_y) == 0 ? 0 : (-gamepad1.left_stick_y + (gamepad1.left_stick_y >0?-0.3:0.3)) * ninjaMultiplier,
                            Math.abs(gamepad1.left_stick_x) == 0 ? 0: (-gamepad1.left_stick_x +(gamepad1.left_stick_x>0?-0.3:0.3)) * ninjaMultiplier,
                            -gamepad1.right_stick_x * 0.7 * (ninjaMultiplier + 0.1)
                    )
            );

            //CYCLING:
            //0: ground/terminal, 1: low, 2: mid, 3: high
            if (cycleUp.wasJustPressed())
                cycleValue++;
            if (cycleDown.wasJustPressed())
                cycleValue--;

            //keeps the integer range between 0 and 3
            if (cycleValue < 0)
                cycleValue = 3;
            if (cycleValue > 3)
                cycleValue = 0;

            if ((cycleDown.wasJustPressed() || cycleUp.wasJustPressed()) && autoActuate) {
                gamepad2.stopRumble();
                if (!gamepad2.isRumbling()) {
                    switch (cycleValue) {
                        case 0:
                            slides.setState(Slides.State.BOTTOM);
//                            deposit.setExtension(Deposit.ExtensionState.RETRACT);
                            break;
                        case 1:
                            gamepad2.runRumbleEffect(customRumbleEffect0);
                            slides.setState(Slides.State.LOW);
                            deposit.setExtension(Deposit.ExtensionState.RETRACT);
                            break;
                        case 2:
                            gamepad2.runRumbleEffect(customRumbleEffect1);
                            slides.setState(Slides.State.MID);
                            deposit.setExtension(Deposit.ExtensionState.FOURTH);
                            break;
                        case 3:
                            gamepad2.runRumbleEffect(customRumbleEffect2);
                            deposit.setExtension(Deposit.ExtensionState.FOURTH);
                            slides.setState(Slides.State.HIGH);
                            break;
                    }
                }
            } else if ((cycleDown.wasJustPressed() || cycleUp.wasJustPressed()) && !autoActuate) {
                gamepad2.stopRumble();
                if (!gamepad2.isRumbling()) {
                    switch (cycleValue) {
                        case 0:
                            break;
                        case 1:
                            gamepad2.runRumbleEffect(customRumbleEffect0);
                            break;
                        case 2:
                            gamepad2.runRumbleEffect(customRumbleEffect1);
                            break;
                        case 3:
                            gamepad2.runRumbleEffect(customRumbleEffect2);
                            break;
                    }
                }
            }

            if (conePickup.wasJustPressed()) {
                slides.setState(Slides.State.PICKUP);
                deposit.setAngle(Deposit.AngleState.CONE_PICKUP);
                deposit.setExtension(Deposit.ExtensionState.HALF);
                turret.setState(Turret.State.ZERO);
            }

            if (turretZero.wasJustPressed()) {
                turret.posAtZero = -turret.encoder.getCurrentPosition();
            }
            //incremental turret control:
            if (gamepad2.right_stick_x > 0)
                turret.setState(Turret.State.RIGHT);
            if (gamepad2.right_stick_x < 0)
                turret.setState(Turret.State.LEFT);
            if (Math.abs(gamepad2.left_stick_x) > 0)
                turret.setState(Turret.State.BACK);

            //manual turret control:
            if (Math.abs(gamepad2.right_stick_y) > 0) {
                turret.setState(Turret.State.MANUAL);
                turret.turretMotor.setPower(gamepad2.right_stick_y * 0.5);
                turretStop = true;
            }
            if (turretStop && gamepad2.right_stick_y == 0) {
                turret.setState(Turret.State.MANUAL);
                turret.turretMotor.setPower(0);
                turretStop = false;
            }
            //manual slides control:
            if (Math.abs(gamepad2.left_stick_y) > 0) {
                slides.setState(Slides.State.MANUAL);
            }
            slides.setPowerManual(gamepad2.left_stick_y);
            //GROUND INTAKE
            if (intakeGround.wasJustPressed()) {
                groundIntake.setState(GroundIntake.State.INTAKING);
            } else if (extakeGround.wasJustPressed()) {
                groundIntake.setState(GroundIntake.State.DEPOSITING);
            } else if (gamepad1.dpad_left) {
                groundIntake.setState(GroundIntake.State.OFF);
            } else if (gamepad1.dpad_right) {
                groundIntake.setState(GroundIntake.State.FAST);
            }

            if (depositTransfer.isDown()) {
                claw.setState(Claw.State.OPEN);
            } else if (intakeTransfer.isDown()) {
                claw.setState(Claw.State.CLOSE);
            }

            //extension
            if (extension.wasJustPressed() && deposit.getExtState() == Deposit.ExtensionState.EXTEND) {
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
            } else if (extension.wasJustPressed() && (deposit.getExtState() == Deposit.ExtensionState.RETRACT
                    || deposit.getExtState() == Deposit.ExtensionState.FOURTH)
                    || deposit.getExtState() == Deposit.ExtensionState.HALF) {
                deposit.setExtension(Deposit.ExtensionState.EXTEND);
            }
            //angle
            if (angle.wasJustPressed() && deposit.getAngState() == Deposit.AngleState.VECTORING) {
                deposit.setAngle(Deposit.AngleState.INTAKE);
            } else if (angle.wasJustPressed() && deposit.getAngState() == Deposit.AngleState.INTAKE) {
                deposit.setAngle(Deposit.AngleState.VECTORING);
            }
            //half extension
            if (half.wasJustPressed() && (deposit.getExtState() == Deposit.ExtensionState.EXTEND
                    || deposit.getExtState() == Deposit.ExtensionState.RETRACT
                    || deposit.getExtState() == Deposit.ExtensionState.FOURTH)) {
                deposit.setExtension(Deposit.ExtensionState.HALF);
            } else if (half.wasJustPressed() && deposit.getExtState() == Deposit.ExtensionState.HALF) {
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
            }

            //auto close claw
            if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 5.75 && slides.slidesLeft.getCurrentPosition() < 100 && distanceSensor) {
                claw.setState(Claw.State.CLOSE);
                distanceSensor = false;
            }
            if (robot.distanceSensor.getDistance(DistanceUnit.CM) > 7) {
                distanceSensor = true;
            }

            //reset
            if (reset.wasJustPressed()) {
                autoActuate = false;
                resetCheck = true;
                resetTimer.reset();
            }
            //turret mid
            if (actuateUp.wasJustPressed()) {
                autoActuate = true;
                transfer = true;
                turretPos = 1;
                transferTimer.reset();
            }
            if (actuateLeft.wasJustPressed()) {
                autoActuate = true;
                transfer = true;
                turretPos = 0;
                transferTimer.reset();
            }
            if (actuateRight.wasJustPressed()) {
                autoActuate = true;
                transfer = true;
                turretPos = 2;
                transferTimer.reset();
            }

            if (cycleMacro.wasJustPressed()) {
                cycleCheck = true;
                cycleTimer.reset();
            }

            //AUTO ALIGN:
           if (autoAlign.wasJustPressed() && turret.detector.getLocation() != Detector.Location.MIDDLE) {
                turret.setState(Turret.State.AUTOALIGN);
            }
            if (turret.detector.getLocation() == Detector.Location.MIDDLE&&turret.getState()==Turret.State.AUTOALIGN) {
                turret.setState(Turret.State.IDLE);
            }

            transferUpdate(cycleValue);
            resetUpdate();
            cycle();


            //TELEMETRY
            telemetry.addData("cycle: ", cycleValue);
            telemetry.addData("turret power: ", turret.turretMotor.getPower());
            telemetry.addData("Auto Align (Aligned < 0.6)", turret.detector.getShift());
            telemetry.addData("Turret State", turret.getState());
            telemetry.addData("Turret", turret.turretMotor.getCurrentPosition());
            telemetry.addData("Slides 1: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("Slides 2: ", slides.slidesRight.getCurrentPosition());
            telemetry.addData("Extension State: ", deposit.getExtState());
            telemetry.addData("Slides State: ", slides.getState());
            telemetry.addData("Auto Actuate: ", autoActuate);
            telemetry.addData("Turret Target Position", turret.getTargetPos());
            telemetry.addData("turret pos at zero: ", turret.posAtZero);
            telemetry.addData("SLIDES LIMIT SWITCH: ", slides.limitState());
            telemetry.addData("SLS VOLTAGE: ", slides.slidesLimitSwitch.getState());
            telemetry.addData("SLIDES POS AT ZERO: ", slides.posAtZero);
            telemetry.addData("distance sensor: ", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    public void transferUpdate(int cycle) {
        if (transfer) {
            if (transferTimer.milliseconds() > 700) {
                if(cycle==1) {
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                } else {
                    deposit.setExtension(Deposit.ExtensionState.FOURTH);
                }
                transfer = false;
            } else if (transferTimer.milliseconds() > 300 || slides.slidesLeft.getCurrentPosition() > 800) {
                switch(turretPos) {
                    case 0:
                        turret.setState(Turret.State.LEFT);
                        break;
                    case 1:
                        turret.setState(Turret.State.BACK);
                        break;
                    case 2:
                        turret.setState(Turret.State.RIGHT);
                        break;
                }
            } else if (transferTimer.milliseconds() > 200) {
                deposit.setAngle(Deposit.AngleState.VECTORING);
            } else if (transferTimer.seconds() > 0) {
                if (cycle == 3) {
                    slides.setState(Slides.State.HIGH);
                } else if (cycle == 1) {
                    slides.setState(Slides.State.LOW);
                } else {
                    slides.setState(Slides.State.MID);
                }
            }
        }
    }

    public void resetUpdate() {
        if (resetCheck) {
            if (resetTimer.milliseconds() > 500) {
                slides.setState(Slides.State.BOTTOM);
                resetCheck = false;
            } else if (resetTimer.seconds() > 0) {
                claw.setState(Claw.State.OPEN);
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
                turret.setState(Turret.State.ZERO);
                deposit.setAngle(Deposit.AngleState.INTAKE);
            }
        }
    }
    public void cycle() {
        if (cycleCheck) {
               if (cycleTimer.milliseconds() > 3000) {
                   resetCheck = true;
                   resetTimer.reset();
                   cycleCheck = false;
                } else if (cycleTimer.milliseconds() > 2500) {
                    claw.setState(Claw.State.OPEN);
                } else if (cycleTimer.milliseconds() > 2000) {
                   deposit.setExtension(Deposit.ExtensionState.EXTEND);
               } else if (cycleTimer.milliseconds() > 1200) {
                    turret.setState(Turret.State.BACK);
                } else if (cycleTimer.milliseconds() > 800) {
                   deposit.setExtension(Deposit.ExtensionState.RETRACT);
                   slides.setState(Slides.State.HIGH);
                    deposit.setAngle(Deposit.AngleState.VECTORING);
                } else if (cycleTimer.milliseconds() > 350) {
                    claw.setState(Claw.State.CLOSE);
                } else if (cycleTimer.milliseconds() > 0) {
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                }
        }
    }
}