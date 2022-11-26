package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class DriverControl extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret turret;
    Detector detector1;
    OpenCvWebcam webcam;
    GamepadEx primary, secondary;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    KeyReader[] keyReaders;
    TriggerReader intakeTransfer, depositTransfer,  intakeGround, extakeGround;
    ButtonReader cycleDown, cycleUp, actuateLeft, actuateUp, actuateRight, turretRight, turretLeft, reset, raiseSlides, lowerSlides, fourBarPrimed, fourBarDeposit, fourBarIntake, turretZero;
    ToggleButtonReader junctionScore, ninjaMode, straightMode, autoAlign;
    Gamepad.RumbleEffect customRumbleEffect0;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect2;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect3;    // Use to build a custom rumble sequence.

    int cycleValue = 0;
    boolean slidesZero = false, turretStop = false;
    boolean autoAlignCheck=false, zeroCheck=false;
    @Override
    public void runOpMode() throws InterruptedException {
        camInit();

        robot = new Robot(hardwareMap);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        keyReaders = new KeyReader[] {
                ninjaMode = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                intakeGround = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),
                extakeGround = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                straightMode = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                
                intakeTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                depositTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.LEFT_TRIGGER),
                fourBarPrimed = new ButtonReader(secondary, GamepadKeys.Button.B),
                fourBarDeposit = new ButtonReader(secondary, GamepadKeys.Button.Y),
                fourBarIntake= new ButtonReader(secondary, GamepadKeys.Button.A),
                actuateRight = new ButtonReader(secondary, GamepadKeys.Button.DPAD_RIGHT),
                actuateLeft = new ButtonReader(secondary,GamepadKeys.Button.DPAD_LEFT),
                actuateUp = new ButtonReader(secondary, GamepadKeys.Button.DPAD_UP),
                reset = new ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN),
                autoAlign = new ToggleButtonReader(secondary, GamepadKeys.Button.X),
                cycleDown = new ButtonReader(secondary, GamepadKeys.Button.LEFT_BUMPER),
                cycleUp = new ButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER)
        //junctionScore: lowers v4b on high junction
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
        fourbar.setState(vfourb.State.PRIMED);
        robot.odoRaise.setPosition(0);
        while (!isStopRequested()) {

            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }
            //DRIVETRAIN
            if (ninjaMode.getState()) {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * 0.5,
                                -gamepad1.left_stick_x * 0.5,
                                -gamepad1.right_stick_x * 0.5
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
            }
            else robot.setWeightedDrivePower(
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
            if(cycleDown.wasJustPressed()||cycleUp.wasJustPressed()){
                gamepad2.stopRumble();
                if(!gamepad2.isRumbling()) {
                    switch (cycleValue) {
                        case 0:
                          //  gamepad2.runRumbleEffect(customRumbleEffect0);
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
                autoAlignCheck=false;
                slides.setState(Slides.State.BOTTOM);
                fourbar.setState(vfourb.State.PRIMED);
                turret.setState(Turret.State.ZERO);
            }

            if(autoAlign.wasJustPressed()){
                autoAlignCheck=!autoAlignCheck;
            }
            //turret left
            if (actuateLeft.wasJustPressed()&&autoAlignCheck==false) {
                switch (cycleValue) {
                    case 0:
                        slides.setState(Slides.State.BOTTOM);
                        fourbar.setState(vfourb.State.PRIMED);
                        turret.setState(Turret.State.LEFT);
                        break;
                    case 1:
                        slides.setState(Slides.State.LOW);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.LEFT);
                        break;
                    case 2:
                        slides.setState(Slides.State.MID);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.LEFT);
                        break;
                    case 3:
                        slides.setState(Slides.State.HIGH);
                        fourbar.setState(vfourb.State.ALIGN_POSITION);
                        turret.setState(Turret.State.LEFT);
                        break;
                }
            }

            //turret right
            if (actuateRight.wasJustPressed()&& !autoAlignCheck) {
                switch (cycleValue) {
                    case 0:
                        slides.setState(Slides.State.BOTTOM);
                        fourbar.setState(vfourb.State.PRIMED);
                        turret.setState(Turret.State.RIGHT);
                        break;
                    case 1:
                        slides.setState(Slides.State.LOW);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.RIGHT);
                        break;
                    case 2:
                        slides.setState(Slides.State.MID);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.RIGHT);
                        break;
                    case 3:
                        slides.setState(Slides.State.HIGH);
                        fourbar.setState(vfourb.State.ALIGN_POSITION);
                        turret.setState(Turret.State.RIGHT);
                        break;
                }
            }

            //turret left
            if (actuateLeft.wasJustPressed()&& !autoAlignCheck) {
                switch (cycleValue) {
                    case 0:
                        slides.setState(Slides.State.BOTTOM);
                        fourbar.setState(vfourb.State.PRIMED);
                        turret.setState(Turret.State.LEFT);
                        break;
                    case 1:
                        slides.setState(Slides.State.LOW);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.LEFT);
                        break;
                    case 2:
                        slides.setState(Slides.State.MID);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.LEFT);
                        break;
                    case 3:
                        slides.setState(Slides.State.HIGH);
                        fourbar.setState(vfourb.State.ALIGN_POSITION);
                        turret.setState(Turret.State.LEFT);
                        break;
                }
            }

            //turret mid
            if (actuateUp.wasJustPressed()&& !autoAlignCheck) {
                switch (cycleValue) {
                    case 0:
                        slides.setState(Slides.State.BOTTOM);
                        fourbar.setState(vfourb.State.PRIMED);
                        turret.setState(Turret.State.ZERO);
                        break;
                    case 1:
                        slides.setState(Slides.State.LOW);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.ZERO);
                        break;
                    case 2:
                        slides.setState(Slides.State.MID);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        turret.setState(Turret.State.ZERO);
                        break;
                    case 3:
                        slides.setState(Slides.State.HIGH);
                        fourbar.setState(vfourb.State.ALIGN_POSITION);
                        turret.setState(Turret.State.ZERO);
                        break;
                }
            }
            if(autoAlignCheck){
                if (detector1.getLocation()== Detector.Location.LEFT && turret.turretMotor.getCurrentPosition() > -390) {
                    turret.setState(Turret.State.LEFT);
                } else if (detector1.getLocation()== Detector.Location.RIGHT && turret.turretMotor.getCurrentPosition() < 390) {
                    turret.setState(Turret.State.RIGHT);
                }else{
                    turret.setState(Turret.State.IDLE);
                }
            }

            //manual turret control:
            if (Math.abs(gamepad2.right_stick_x) > 0) {
                turret.setState(Turret.State.MANUAL);
                turret.turretMotor.setPower(gamepad2.right_stick_x);
                turretStop = true;
            }
            if (turretStop && gamepad2.right_stick_x == 0) {
                turret.setState(Turret.State.MANUAL);
                turret.turretMotor.setPower(0);
                turretStop = false;
            }
            //manual slides control:
            if (Math.abs(gamepad2.left_stick_y) > 0) {
                slides.setState(Slides.State.MANUAL);
                slides.slidesLeft.setPower(gamepad2.left_stick_y);
                slides.slidesRight.setPower(gamepad2.left_stick_y);
                    slidesZero = true;
            }
            if (slidesZero && gamepad2.left_stick_y == 0) {
                slides.setState(Slides.State.MANUAL);
                slides.slidesLeft.setPower(0);
                slides.slidesRight.setPower(0);
                slidesZero = false;
            }

            //GROUND INTAKE
            if (intakeGround.isDown()) {
                groundIntake.setState(GroundIntake.State.INTAKING);
            } else if (extakeGround.isDown()){
                groundIntake.setState(GroundIntake.State.DEPOSITING);
            } else {
                groundIntake.setState(GroundIntake.State.OFF);
            }

            //robot.groundIntake.update();

            //SCORING:
//            if (junctionScore.wasJustPressed() && slides.getState() == Slides.State.HIGH) {
//                slides.setState(Slides.State.HIGH_DROP);
//                fourbar.setState(vfourb.State.DEPOSIT_POSITION);
//            }
//            if (junctionScore.wasJustPressed() && slides.getState() == Slides.State.MID){
////                slides.setState(Slides.State.MID_DROP);
//                fourbar.setState(vfourb.State.DEPOSIT_POSITION);
//            }
//            if (junctionScore.wasJustPressed() && slides.getState() == Slides.State.LOW) {
////                slides.setState(Slides.State.LOW_DROP);
//            }
            //DEPOSIT:
            if (depositTransfer.isDown()) {
                intake.setState(Intake.State.INTAKING);
            } else if (intakeTransfer.isDown()) {
                intake.setState(Intake.State.DEPOSITING);
            } else {
                intake.setState(Intake.State.OFF);
            }

            //V4B:
            if (fourBarIntake.wasJustPressed()) {
                fourbar.setState(vfourb.State.INTAKE_POSITION);
                }
            if (fourBarDeposit.wasJustPressed()) {
                fourbar.setState(vfourb.State.DEPOSIT_POSITION);
            }
            if (fourBarPrimed.wasJustPressed()) {
                fourbar.setState(vfourb.State.PRIMED);
            }

            //TELEMETRY
            telemetry.addData("cycle: ", cycleValue);
            telemetry.addData("turret power: ", turret.turretMotor.getPower());
            telemetry.addData("AutoAlign", autoAlignCheck);
            telemetry.addData("Turret", turret.getState());
            telemetry.addData("Turret", turret.turretMotor.getCurrentPosition());
            telemetry.addData("Slides 1: ", slides.slidesLeft.getPower());
            telemetry.addData("Slides 2: ", slides.slidesRight.getPower());
//            telemetry.addData("Ground Intake Sensor", groundIntake.sensorVal());
            telemetry.addData("V4B State: ",fourbar.getState());
            telemetry.addData("Slides State: ", slides.getState());
            telemetry.update();


        }
        turret.stop();
    }

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
        dashboard.startCameraStream(webcam, 30);
        telemetry.addLine("waiting for start");
        telemetry.update();
    }
}