package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.BackgroundTasks;
import org.firstinspires.ftc.teamcode.util.Context;

@Config
@TeleOp(name = "1. Driver Practice")
public class DriverControl extends LinearOpMode {
    Robot robot;
    Slides slides;
    Deposit deposit;
    GroundIntake groundIntake;
    Claw claw;
    Turret turret;
    BackgroundTasks hardware;
    TelemetryPacket packet;
    GamepadEx primary, secondary;
    double timer;
    boolean transfer = false;
    int stackValue = 5;
    boolean resetCheck, cycleCheck = false;
    int turretPos = -1; //0 = left, 1 = back, 2 = right
    public static double powerOffsetTurning = 0.25;
    double ninjaMultiplier = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean wristClockwork = false;
    ElapsedTime wristClock = new ElapsedTime();
    KeyReader[] keyReaders;
    TriggerReader intakeTransfer, depositTransfer, actuateUp;
    ButtonReader stackUp, stackDown, cycleMacro, cycleDown, cycleUp, actuateLeft,
            intakeGround, extakeGround, actuateRight, reset, half, angle, extension, turretZero, slidesReset;
    ToggleButtonReader alignerAdjust, straightMode;

    Gamepad.RumbleEffect customRumbleEffect0;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect2;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect3;    // Use to build a custom rumble sequence.

    int cycleValue = 0;
    boolean slidesZero = false, turretStop = false;
    boolean autoActuate = false;
    public double resetTime = 1;
    public double transferTime = 1, transferTurretTime = 500;
    public double WRIST_DELAY = 0;
    public boolean distanceSensor = false;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime resetTimer = new ElapsedTime();
    ElapsedTime cycleTimer = new ElapsedTime();
    TrajectorySequence cycleIntake, cycleDropOff;
    public static double sideOdomPos = 0.33;
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
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        keyReaders = new KeyReader[]{
                alignerAdjust = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                intakeGround = new ToggleButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                extakeGround = new ToggleButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                straightMode = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                turretZero = new ToggleButtonReader(primary, GamepadKeys.Button.X),
                cycleMacro = new ButtonReader(primary, GamepadKeys.Button.B),
                stackDown = new ButtonReader(primary, GamepadKeys.Button.A),
                stackUp = new ButtonReader(primary, GamepadKeys.Button.Y),
                intakeTransfer = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),

                actuateRight = new ButtonReader(secondary, GamepadKeys.Button.DPAD_RIGHT),
                actuateLeft = new ButtonReader(secondary, GamepadKeys.Button.DPAD_LEFT),
                actuateUp = new TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                depositTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.LEFT_TRIGGER),
                reset = new ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN),
                half = new ButtonReader(secondary, GamepadKeys.Button.B),
                angle = new ButtonReader(secondary, GamepadKeys.Button.Y),
                slidesReset = new ButtonReader(secondary, GamepadKeys.Button.X),
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
        Trajectory cycleIntake = robot.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(2, 0),robot.getVelocityConstraint(15, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{
                    turret.setState(Turret.State.ZERO);
                    deposit.setExtension(Deposit.ExtensionState.EXTEND);
                })

                .build();

        Trajectory cycleDrop = robot.trajectoryBuilder(cycleIntake.end())
                .lineToConstantHeading(new Vector2d(-0.5, 0),robot.getVelocityConstraint(15, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{
                    slides.setState(Slides.State.HIGH);
                })
                .addTemporalMarker(0.1, ()->{
                    turret.setState(Turret.State.BACK);
                    claw.setPoleState(Claw.Pole.DOWN);
                })
                .build();
        /*
        Trajectory align = robot.trajectoryBuilder(cycleDrop.end())

                .lineToConstantHeading(new Vector2d(0, 0),robot.getVelocityConstraint(10, 5.939, 13.44),
                        robot.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{
                    turret.setState(Turret.State.ZERO);
                    slides.setState(Slides.State.BOTTOM);

                })

                .build();
*/

        waitForStart();
        //robot.turretCamera.pauseViewport();
        gamepad1.setLedColor(100, 79, 183, 120000); //blue
        gamepad2.setLedColor(147, 112, 219, 120000); //light purple
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.TELE_INTAKE);
        claw.setState(Claw.State.OPEN);
        slides.setState(Slides.State.BOTTOM);
        robot.midOdo.setPosition(0);
        robot.sideOdo.setPosition(1);
        turret.setState(Turret.State.ZERO);
        while (opModeIsActive()) {
            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
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
                            Math.abs(gamepad1.left_stick_y) == 0 ? 0 : (-gamepad1.left_stick_y + (gamepad1.left_stick_y > 0?-0.25:0.25)) * (ninjaMultiplier - 0.3),
                            Math.abs(gamepad1.left_stick_x) <= 0.25 ? 0: (-gamepad1.left_stick_x +(gamepad1.left_stick_x>0?-0.25:0.25)) * (ninjaMultiplier - 0.3),
                            Math.abs(gamepad1.right_stick_x) == 0 ? 0: ((-0.65*Math.pow(gamepad1.right_stick_x, 3)+(gamepad1.right_stick_x>0?-powerOffsetTurning : powerOffsetTurning)) * (ninjaMultiplier))
                    )
            );

            //CYCLING:
            //0: ground/terminal, 1: low, 2: mid, 3: high
            if (cycleUp.wasJustPressed())
                cycleValue++;
            if (cycleDown.wasJustPressed())
                cycleValue--;

            if (stackUp.wasJustPressed()) {
                stackValue++;
                if (stackValue < 0)
                    stackValue = 4;
                if (stackValue > 4)
                    stackValue = 0;
                switch (stackValue) {
                    case 0:
                        slides.setState(Slides.State.CYCLE0);
                        break;
                    case 1:
                        slides.setState(Slides.State.CYCLE1);
                        break;
                    case 2:
                        slides.setState(Slides.State.CYCLE2);
                        break;
                    case 3:
                        slides.setState(Slides.State.CYCLE3);
                        break;
                    case 4:
                        slides.setState(Slides.State.CYCLE4);
                        break;
                }
            }
            if (stackDown.wasJustPressed()) {
                stackValue--;
                if (stackValue < 0)
                    stackValue = 4;
                if (stackValue > 4)
                    stackValue = 0;
                switch (stackValue) {
                    case 0:
                        slides.setState(Slides.State.CYCLE0);
                        break;
                    case 1:
                        slides.setState(Slides.State.CYCLE1);
                        break;
                    case 2:
                        slides.setState(Slides.State.CYCLE2);
                        break;
                    case 3:
                        slides.setState(Slides.State.CYCLE3);
                        break;
                    case 4:
                        slides.setState(Slides.State.CYCLE4);
                        break;
                }
            }


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
                            claw.setPoleState(Claw.Pole.TELE_UP);
                            break;
                        case 1:
                            gamepad2.runRumbleEffect(customRumbleEffect0);
                            slides.setState(Slides.State.LOW);
                            deposit.setExtension(Deposit.ExtensionState.RETRACT);
                            claw.setPoleState(Claw.Pole.TELE_UP);
                            break;
                        case 2:
                            gamepad2.runRumbleEffect(customRumbleEffect1);
                            slides.setState(Slides.State.MID);
                            deposit.setExtension(Deposit.ExtensionState.TELE_FOURTH);
                            claw.setPoleState(Claw.Pole.TELE_DOWN);
                            break;
                        case 3:
                            gamepad2.runRumbleEffect(customRumbleEffect2);
                            deposit.setExtension(Deposit.ExtensionState.TELE_FOURTH);
                            slides.setState(Slides.State.HIGH);
                            claw.setPoleState(Claw.Pole.TELE_DOWN);
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

            if (turretZero.wasJustPressed()) {
                turret.posAtZero = -turret.encoder.getCurrentPosition();
            }

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

            if (depositTransfer.wasJustPressed()) {
                wristClockwork = true;
                wristClock.reset();

                deposit.setAngle(Deposit.AngleState.TELE_INTAKE);
                if (slides.slidesLeft.getCurrentPosition() - slides.posAtZero > 1000)
                    claw.setPoleState(Claw.Pole.TELE_DEPOSIT);
            } else if (intakeTransfer.wasJustPressed()) {
                claw.setState(Claw.State.CLOSE);
            }

            if (wristClockwork&&wristClock.milliseconds()>WRIST_DELAY){
                claw.setState(Claw.State.OPEN);
                wristClockwork = false;
            }
            //extension
            if (extension.wasJustPressed() && deposit.getExtState() == Deposit.ExtensionState.EXTEND) {
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
            } else if (extension.wasJustPressed() && (deposit.getExtState() == Deposit.ExtensionState.RETRACT
                    || deposit.getExtState() == Deposit.ExtensionState.TELE_FOURTH)
                    || deposit.getExtState() == Deposit.ExtensionState.HALF) {
                deposit.setExtension(Deposit.ExtensionState.EXTEND);
            }
            //angle
            if (angle.wasJustPressed() && deposit.getAngState() == Deposit.AngleState.VECTORING) {
                deposit.setAngle(Deposit.AngleState.TELE_INTAKE);
            } else if (angle.wasJustPressed() && deposit.getAngState() == Deposit.AngleState.TELE_INTAKE) {
                deposit.setAngle(Deposit.AngleState.VECTORING);
            }
            //half extension
            if (half.wasJustPressed()) {
                claw.setState(Claw.State.CLOSE);
            }

            //auto close claw
            if (robot.distanceSensor.getDistance(DistanceUnit.CM) <= 8 &&
                    slides.slidesLeft.getCurrentPosition() - slides.posAtZero < 80 && distanceSensor) {
                claw.setState(Claw.State.CLOSE);
                distanceSensor = false;
            }
            if (robot.distanceSensor.getDistance(DistanceUnit.CM) > 8) {
                distanceSensor = true;
            }

            //reset
            if (reset.wasJustPressed()) {
                autoActuate = false;
                resetCheck = true;
                resetTimer.reset();
            }

            if (slidesReset.wasJustPressed()) {
                slides.posAtZero = slides.slidesLeft.getCurrentPosition();
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

            if (alignerAdjust.wasJustPressed()) {
                claw.setPoleState(Claw.Pole.TELE_DEPOSIT);
            }

            if (cycleMacro.wasJustPressed()) {
                //robot.turretCamera.resumeViewport();
                robot.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
                robot.followTrajectory(cycleIntake);
                intake();
                robot.followTrajectory(cycleDrop);
                dropOff();
                //robot.turretCamera.pauseViewport();
            }

//            if (stackDown.wasJustPressed() && sideOdomPos == 0.33) { //up
//                sideOdomPos = 0.65;
//                robot.midOdo.setPosition(0);
//                robot.sideOdo.setPosition(sideOdomPos);
//                Context.hallEffectEnabled=false;
//                //turret.isAuto=false;
//                //robot.turretCamera.pauseViewport();
//            } else if (stackDown.wasJustPressed() && sideOdomPos == 0.65) { //down
//                sideOdomPos = 0.33;
//                robot.midOdo.setPosition(sideOdomPos);
//                robot.sideOdo.setPosition(sideOdomPos);
//                //robot.turretCamera.resumeViewport();
//                //robot.turret.setHall(Turret.Hall.ON);
//            }
            

            //AUTO ALIGN:
//           if (stackUp.wasJustPressed() && turret.detector.getLocation() != AlignerAuto.Location.MIDDLE) {
//                turret.setState(Turret.State.stackUp);
//            }
//            if (turret.detector.getLocation() == AlignerAuto.Location.MIDDLE&&turret.getState()==Turret.State.stackUp) {
//                turret.setState(Turret.State.IDLE);
//            }

            

            transferUpdate(cycleValue);
            resetUpdate();
            cycle();


            //TELEMETRY
            telemetry.addData("cycle: ", cycleValue);
            telemetry.addData("turret power: ", turret.turretMotor.getPower());
            telemetry.addData("Turret State", turret.getState());
            telemetry.addData("Turret", turret.turretMotor.getCurrentPosition());
            telemetry.addData("Slides 1: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("Slides 2: ", slides.slidesRight.getCurrentPosition());
            telemetry.addData("Extension State: ", deposit.getExtState());
            telemetry.addData("Slides State: ", slides.getState());
            telemetry.addData("Auto Actuate: ", autoActuate);
            telemetry.addData("Turret Target Position", turret.getTargetPos());
            telemetry.addData("turret pos at zero: ", turret.posAtZero);
           // telemetry.addData("SLS VOLTAGE: ", slides.slidesLimitSwitch.getState());
            telemetry.addData("SLIDES POS AT ZERO: ", slides.posAtZero);
//            telemetry.addData("distance sensor: ", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }



    public void transferUpdate(int cycle) {
        if (transfer && cycle != 0) {
            if (transferTimer.milliseconds() > 1000) {
                if(cycle==1) {
                    deposit.setExtension(Deposit.ExtensionState.RETRACT);
                } else {
                    deposit.setExtension(Deposit.ExtensionState.TELE_FOURTH);
                }
                transfer = false;
            } else if (transferTimer.milliseconds() > 300 || slides.slidesLeft.getCurrentPosition() - slides.posAtZero > 500) {
                deposit.setAngle(Deposit.AngleState.VECTORING);
                if (cycle != 1) {
                    claw.setPoleState(Claw.Pole.TELE_DOWN);
                }
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
            } else if (transferTimer.seconds() > 0) {
                switch (cycleValue) {
                    case 0:
                        slides.setState(Slides.State.BOTTOM);
                        break;
                    case 1:
                        slides.setState(Slides.State.LOW);
                        break;
                    case 2:
                        slides.setState(Slides.State.MID);
                        break;
                    case 3:
                        slides.setState(Slides.State.HIGH);
                        break;
                }
            }
        }
    }

    public void resetUpdate() {
        if (resetCheck) {
//            turret.setState(Turret.State.ZERO);
//            claw.setState(Claw.State.OPEN);
//            deposit.setExtension(Deposit.ExtensionState.RETRACT);
//            deposit.setAngle(Deposit.AngleState.TELE_INTAKE);
//            claw.setPoleState(Claw.Pole.UP);
//            if (slides.slidesLeft.getCurrentPosition() - slides.posAtZero < 1200) {
//                turret.setState(Turret.State.ZERO);
//                if (resetTimer.milliseconds() > 500) {
//                    slides.setState(Slides.State.BOTTOM);
//                    resetCheck = false;
//                }
//            } else {
//                turret.setState(Turret.State.ZERO);
//                claw.setState(Claw.State.OPEN);
//                deposit.setExtension(Deposit.ExtensionState.RETRACT);
//                deposit.setAngle(Deposit.AngleState.TELE_INTAKE);
//                if (resetTimer.milliseconds() > 200) {
//                    slides.setState(Slides.State.BOTTOM);
//                    resetCheck = false;
//                }
//            }
            if (resetTimer.milliseconds() > 1000) {
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
                resetCheck = false;
            } else if (resetTimer.milliseconds() > 0) {
                turret.setState(Turret.State.ZERO);
                claw.setState(Claw.State.OPEN);
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
                deposit.setAngle(Deposit.AngleState.TELE_INTAKE);
                claw.setPoleState(Claw.Pole.TELE_UP);
                if (slides.slidesLeft.getCurrentPosition() - slides.posAtZero > 1000 && resetTimer.milliseconds() > 300) {
                    slides.setState(Slides.State.BOTTOM);
                }
                if (slides.slidesLeft.getCurrentPosition() - slides.posAtZero < 1000 && resetTimer.milliseconds() > 500) {
                    slides.setState(Slides.State.BOTTOM);
                }
            }
        }
    }
    public void cycle() {
        if (cycleCheck) {
//            robot.midOdo.setPosition(0.33);
//            robot.sideOdo.setPosition(0.33);
               if (cycleTimer.milliseconds() > 2800) {
                   resetCheck = true;
                   resetTimer.reset();
                   cycleCheck = false;
                } else if (cycleTimer.milliseconds() > 2400) {
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
//            robot.followTrajectorySequenceAsync(cycleIntake);
        }
        //else {
//            robot.midOdo.setPosition(0);
//            robot.sideOdo.setPosition(0.65);
//        }
    }
    public void dropOff(){
        timer = System.currentTimeMillis();
        turret.setState(Turret.State.BACK);
        while(System.currentTimeMillis()-475 < timer){
            robot.update();
        }
        deposit.setExtension(Deposit.ExtensionState.EXTEND);
        claw.setPoleState(Claw.Pole.DEPOSIT);
        //turret.setState(Turret.State.stackUp);
        timer = System.currentTimeMillis();

        while(System.currentTimeMillis()-300 < timer){
            /*
            if (turret.stackUp.getLocation() == AlignerAuto.Location.MIDDLE&&turret.getState()==Turret.State.stackUp) {
                turret.setState(Turret.State.IDLE);
            }*/
            robot.update();
        }
        claw.setState(Claw.State.OPEN);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-150< timer){
            robot.update();
        }
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        deposit.setAngle(Deposit.AngleState.TELE_INTAKE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-75< timer){
            robot.update();
        }
        turret.setState(Turret.State.ZERO);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-60< timer){
            robot.update();
        }
        claw.setPoleState(Claw.Pole.UP);
        deposit.setExtension(Deposit.ExtensionState.SLIGHT);
        slides.setState(Slides.State.BOTTOM);

    }
    public void intake(){

        claw.setState(Claw.State.CLOSE);
        timer = System.currentTimeMillis();
        while(System.currentTimeMillis()-200< timer){
            robot.update();
        }
        deposit.setExtension(Deposit.ExtensionState.RETRACT);
        slides.setState(Slides.State.HIGH);
        deposit.setAngle(Deposit.AngleState.VECTORING);

    }
}