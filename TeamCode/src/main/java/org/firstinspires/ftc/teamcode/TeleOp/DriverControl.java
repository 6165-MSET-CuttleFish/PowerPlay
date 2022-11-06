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
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    TriggerReader intakeTransfer, intakeGround, extakeGround, depositTransfer;
    ButtonReader turretRight, turretLeft, reset, raiseSlides, lowerSlides, fourBarPrimed, fourBarDeposit, fourBarIntake;
    ToggleButtonReader highJunctionScore, ninjaMode, autoAlign;
    int slidesTargetPosition = 0;
    boolean autoAlignCheck=false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;



        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        camInit();


        keyReaders = new KeyReader[] {
                ninjaMode = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                intakeGround = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                extakeGround = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),
                
                intakeTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                depositTransfer = new TriggerReader(secondary, GamepadKeys.Trigger.LEFT_TRIGGER),
                fourBarPrimed = new ButtonReader(secondary, GamepadKeys.Button.B),
                fourBarDeposit = new ButtonReader(secondary, GamepadKeys.Button.Y),
                fourBarIntake= new ButtonReader(secondary, GamepadKeys.Button.A),
                turretRight = new ButtonReader(secondary, GamepadKeys.Button.DPAD_RIGHT),
                turretLeft = new ButtonReader(secondary,GamepadKeys.Button.DPAD_LEFT),
                raiseSlides = new ButtonReader(secondary, GamepadKeys.Button.DPAD_UP),
                lowerSlides = new ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN),
                autoAlign = new ToggleButtonReader(secondary, GamepadKeys.Button.X),
                highJunctionScore = new ToggleButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER)
        };
        waitForStart();
        //slides.setState(Slides.State.INTAKE);
        fourbar.setState(vfourb.State.PRIMED);
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
                                gamepad1.left_stick_x * 0.5,
                                -gamepad1.right_stick_x * 0.5
                        )
                );
            }
            else robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (intakeTransfer.isDown()) {
                intake.setState(Intake.State.INTAKING);
            }
            else if (depositTransfer.isDown()) {
                intake.setState(Intake.State.DEPOSITING);
            }
            else {
                intake.setState(Intake.State.OFF);
            }
            if (raiseSlides.wasJustPressed()) {
                switch(slides.getState()) {
                    case BOTTOM:
                        slides.setState(Slides.State.LOW);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        break;
                    case LOW:
                        slides.setState(Slides.State.MID);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        break;
                    case HIGH_DROP:
                    case MID:
                        slides.setState(Slides.State.HIGH);
                        fourbar.setState(vfourb.State.HIGH_POSITION);
                        break;
                }

            }

            if (lowerSlides.wasJustPressed()) {
                switch(slides.getState()) {
                    case HIGH_DROP:
                    case HIGH:
                        slides.setState(Slides.State.MID);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        break;
                    case MID:
                        slides.setState(Slides.State.LOW);
                        fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                        break;
                    case LOW:
                        slides.setState(Slides.State.BOTTOM);
                        fourbar.setState(vfourb.State.INTAKE_POSITION);
                        break;
                }
            }

            //TURRET
            turret.position=turret.turretMotor.getCurrentPosition()-turret.prevPositionReset;
            if(autoAlign.wasJustPressed()){
                turret.zero();
            }
            if(turretLeft.isDown()&& turret.turretMotor.getCurrentPosition() > -390){
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(0.25);
                }else if(turretRight.isDown()&& turret.turretMotor.getCurrentPosition() < 390){
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(-0.25);
                }else {
                    turret.turretMotor.setTargetPosition(turret.position);
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            telemetry.addData("AuoAlign", autoAlignCheck);
            telemetry.addData("Pos", detector1.getLocation());
//            if(!autoAlignCheck){
//                if(turretLeft.isDown()&& turret.turretMotor.getCurrentPosition() > -390){
//                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    turret.turretMotor.setPower(0.75);
//                }else if(turretRight.isDown()&& turret.turretMotor.getCurrentPosition() < 390){
//                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    turret.turretMotor.setPower(-0.75);
//                }else {
//                    turret.turretMotor.setTargetPosition(turret.position);
//                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }else if((autoAlignCheck)&&(fourbar.getState()==vfourb.State.PRIMED)){
//                if(detector1.getLocation()== Detector.Location.LEFT){
//                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    turret.turretMotor.setPower(0.125);
//                }else if(detector1.getLocation()== Detector.Location.RIGHT){
//                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    turret.turretMotor.setPower(-0.125);
//                }else{
//                    turret.turretMotor.setTargetPosition(turret.position);
//                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }

            /*else if((autoAlignCheck)&&(fourbar.getState()==vfourb.State.DEPOSIT_POSITION)){
                if(detector2.getLocation()== Detector.Location.LEFT){
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(0.125);
                }else if(detector2.getLocation()== Detector.Location.RIGHT){
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(-0.125);
                }else{
                    turret.turretMotor.setTargetPosition(turret.position);
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }*/
            if(turret.magnetic.isPressed()){
                turret.prevPositionReset=turret.position;
                turret.position=0;
            }
            //GROUND INTAKE
            if (intakeGround.isDown()) {
                robot.groundLeft.setPower(-1);
                robot.groundRight.setPower(-1);
            }
            else if (extakeGround.isDown()){
                robot.groundLeft.setPower(1);
                robot.groundRight.setPower(1);
            } else {
                robot.groundLeft.setPower(0);
                robot.groundRight.setPower(0);
            }
            //robot.groundIntake.update();

            //DEPOSIT:
            if (highJunctionScore.wasJustPressed()) {
                slides.setState(Slides.State.HIGH_DROP);
            }
            if (depositTransfer.isDown()) {
                intake.setState(Intake.State.DEPOSITING);
            }
            else if (intakeTransfer.isDown()) {
                intake.setState(Intake.State.INTAKING);
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
            telemetry.addData("V4B State: ",fourbar.getState());
            //telemetry.addData("1 Ticks: ", robot.slides.slidesLeft.getCurrentPosition());
            //telemetry.addData("2 Ticks: ", robot.slides.slidesLeft.getCurrentPosition());
            telemetry.update();


        }
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