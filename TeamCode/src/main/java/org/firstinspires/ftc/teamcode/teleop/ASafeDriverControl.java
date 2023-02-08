package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotTemp;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.transfer.Intake;
import org.firstinspires.ftc.teamcode.modules.transfer.vfourb;
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
    boolean resetCheck = false;
    boolean diagonal = false;
    boolean left = true;
    boolean right = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    KeyReader[] keyReaders;
    TriggerReader intakeTransfer, depositTransfer;
    ButtonReader cycleDown, cycleUp, actuateLeft, intakeGround, extakeGround, actuateUp, actuateRight, turretRight, turretLeft, reset, raiseSlides, lowerSlides, half, angle, extension, turretZero, stackPickup;
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
    public boolean turretCheck, slidesCheck = false;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime resetTimer = new ElapsedTime();

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

                intakeTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                depositTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.LEFT_TRIGGER),
                half = new ButtonReader(secondary, GamepadKeys.Button.B),
                angle = new ButtonReader(secondary, GamepadKeys.Button.Y),
                stackPickup = new ButtonReader(secondary, GamepadKeys.Button.X),
                extension = new ButtonReader(secondary, GamepadKeys.Button.A),
                actuateRight = new ButtonReader(secondary, GamepadKeys.Button.DPAD_RIGHT),
                actuateLeft = new ButtonReader(secondary, GamepadKeys.Button.DPAD_LEFT),
                actuateUp = new ButtonReader(secondary, GamepadKeys.Button.DPAD_UP),
                reset = new ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN),
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

        waitForStart();
        slides.setState(Slides.State.BOTTOM);
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.INTAKE);
        claw.setState(Claw.State.OPEN);
        robot.midOdo.setPosition(0);
        robot.sideOdo.setPosition(0.65);
        turret.setState(Turret.State.ZERO);
        while (opModeIsActive()) {
            //slides.update();
            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }

            //DRIVETRAIN
            if (ninjaMode.getState()) {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * 0.5,
                                -gamepad1.left_stick_x * 0.875,
                                -gamepad1.right_stick_x * 0.6
                        )
                );
            } else if (straightMode.getState()) {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * 0.5,
                                0,
                                -gamepad1.right_stick_x * 0.5
                        )
                );
            } else robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
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
                            deposit.setExtension(Deposit.ExtensionState.RETRACT);
                            break;
                        case 1:
                            gamepad2.runRumbleEffect(customRumbleEffect0);
                            slides.setState(Slides.State.LOW);
                            deposit.setExtension(Deposit.ExtensionState.RETRACT);
                            break;
                        case 2:
                            gamepad2.runRumbleEffect(customRumbleEffect1);
                            slides.setState(Slides.State.MID);
                            deposit.setExtension(Deposit.ExtensionState.EXTEND);
                            break;
                        case 3:
                            gamepad2.runRumbleEffect(customRumbleEffect2);
                            deposit.setExtension(Deposit.ExtensionState.EXTEND);
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
            //setting states based off of cycleValue

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
                transferTimer.reset();
            }

            if (stackPickup.wasJustPressed()) {
                slides.slidesRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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
                slides.setPowerManual(-gamepad2.left_stick_y);
                //slides.setPowerManual(gamepad2.left_stick_y);
                slidesZero = true;
            }
            if (slidesZero && gamepad2.left_stick_y == 0) {
                slides.setState(Slides.State.MANUAL);
                slides.setPowerManual(-gamepad2.left_stick_y);
                slidesZero = false;
            }

            //GROUND INTAKE
            if (intakeGround.wasJustPressed()) {
                groundIntake.setState(GroundIntake.State.INTAKING);
            } else if (extakeGround.wasJustPressed()) {
                groundIntake.setState(GroundIntake.State.DEPOSITING);
            } else if (gamepad1.dpad_left) {
                groundIntake.setState(GroundIntake.State.OFF);
            }

            if (depositTransfer.isDown()) {
                claw.setState(Claw.State.OPEN);
            } else if (intakeTransfer.isDown()) {
                claw.setState(Claw.State.CLOSE);
            }
            telemetry.addData("Deposit", depositTransfer.isDown());

            //horizontal slides:
            //extension
            if (extension.wasJustPressed() && deposit.getExtState() == Deposit.ExtensionState.EXTEND) {
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
            } else if (extension.wasJustPressed() && deposit.getExtState() == Deposit.ExtensionState.RETRACT) {
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
                    || deposit.getExtState() == Deposit.ExtensionState.RETRACT)) {
                deposit.setExtension(Deposit.ExtensionState.HALF);
            } else if (half.wasJustPressed() && deposit.getExtState() == Deposit.ExtensionState.HALF) {
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
            }
            if (gamepad2.touchpad) {
                diagonal = !diagonal;
            }
            if (diagonal) {
                if (actuateLeft.wasJustPressed()) {
                    left = true;
                }
                if (actuateRight.wasJustPressed()) {
                    left = false;
                }
            }
            transferUpdate(cycleValue, diagonal, left);
            resetUpdate();
            //turret.update();
            //slides.update();
            //TELEMETRY
            telemetry.addData("cycle: ", cycleValue);
            telemetry.addData("turret power: ", turret.turretMotor.getPower());
            telemetry.addData("Turret", turret.getState());
            telemetry.addData("Turret", turret.turretMotor.getCurrentPosition());
            telemetry.addData("Slides 1: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("Slides 2: ", slides.slidesRight.getCurrentPosition());
            telemetry.addData("Extension State: ", deposit.getExtState());
            telemetry.addData("Slides State: ", slides.getState());
            telemetry.addData("Auto Actuate: ", autoActuate);
            telemetry.addData("Claw: ", claw.getState());
            telemetry.addData("Kai did dumb dumb(turret ideal) ", turret.getTargetPos());
            telemetry.addData("turret pos at zero: ", turret.posAtZero);
            telemetry.addData("turret pos at he: ", turret.hallEffect.getVoltage());
            telemetry.addData("Motor Speeds: ", slides.getOuput());
            telemetry.addData("SLIDES LIMIT SWITCH: ", slides.limitState());
            telemetry.addData("SLIDES POS AT ZERO: ", slides.posAtZero);



            telemetry.update();
            //turret.update();
        }
    }

    public void transferUpdate(int cycle, boolean diagonal, boolean left) {
        if (transfer && !diagonal) {
            if (transferTimer.milliseconds() > 400) {
                if (cycle == 3) {
                    slides.setState(Slides.State.HIGH);
                }
                if(cycle==1){
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                }else
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                transfer = false;
            } else if (transferTimer.milliseconds() > 200) {
                turret.setState(Turret.State.BACK);

            } else if (transferTimer.seconds() > 0) {
                deposit.setAngle(Deposit.AngleState.VECTORING);
                if (cycle == 1) {
                    slides.setState(Slides.State.LOW);
                } else {
                    slides.setState(Slides.State.MID);
                }
            }
        } else if (transfer && diagonal) {
            if (transferTimer.milliseconds() > 400) {
                if (cycle == 3) {
                    slides.setState(Slides.State.HIGH);
                }
                if(cycle==1){
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                }else
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                transfer = false;
            } else if (transferTimer.milliseconds() > 200) {
                if (left)
                    turret.setState(Turret.State.LEFT_DIAGONAL);
                if (!left)
                    turret.setState(Turret.State.RIGHT_DIAGONAL);

            } else if (transferTimer.seconds() > 0) {
                deposit.setAngle(Deposit.AngleState.VECTORING);
                if (cycle == 1) {
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
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
                turret.setState(Turret.State.ZERO);
                deposit.setAngle(Deposit.AngleState.INTAKE);
                claw.setState(Claw.State.OPEN);
            }
        }
    }
}