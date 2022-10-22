package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.Turret.Turret;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;

@TeleOp
public class DriverControl extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    GroundIntake groundIntake;
    Turret turret;
    GamepadEx primary, secondary;
    KeyReader[] keyReaders;
    TriggerReader intakeButton, activeGround, deposit;
    ButtonReader slidesHigh, turretRight, turretLeft, reset, raiseSlides, lowerSlides;
    ToggleButtonReader  ninjaMode;
    int slidesTargetPosition = 0, turretTargetPosition = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        intake = robot.intake;
        slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        slides.slidesLeft.setTargetPosition(0);
        slides.slidesRight.setTargetPosition(0);
        keyReaders = new KeyReader[] {
                intakeButton = new TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                ninjaMode = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                //slidesHigh = new ButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                turretRight = new ButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER),
                turretLeft = new ButtonReader(secondary,GamepadKeys.Button.LEFT_BUMPER),
                deposit = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),

                reset = new ButtonReader(primary, GamepadKeys.Button.B),
                raiseSlides = new ButtonReader(secondary, GamepadKeys.Button.DPAD_UP),
                lowerSlides = new ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN),
                activeGround = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),
        };
        waitForStart();
        //    slides.setState(Slides.State.INTAKE);
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
            } else robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (intakeButton.isDown()) {
                intake.setState(Intake.State.INTAKING);
                fourbar.setState(vfourb.State.INTAKE_POSITION);
            }
            else if (deposit.isDown()) {
                intake.setState(Intake.State.DEPOSITING);
            }
            else {
                intake.setState(Intake.State.OFF);
            }
            if (slidesHigh.wasJustPressed()) {
                //slides.setState(Slides.State.HIGH);
                fourbar.setState(vfourb.State.DEPOSIT_POSITION);
            }

            //SLIDES
            if (Math.abs(slides.slidesLeft.getCurrentPosition())  >= slidesTargetPosition - 10
                    && Math.abs(slides.slidesLeft.getCurrentPosition()) <= slidesTargetPosition + 10){
                slides.slidesLeft.setPower(0);
                slides.slidesRight.setPower(0);
            }
            else if(Math.abs(slides.slidesLeft.getCurrentPosition())  <= slidesTargetPosition - 10){
                slides.slidesLeft.setPower(0.5);
                slides.slidesRight.setPower(0.5);
            }
            else if(Math.abs(slides.slidesLeft.getCurrentPosition())  >= slidesTargetPosition + 10){
                slides.slidesLeft.setPower(-0.5);
                slides.slidesRight.setPower(-0.5);
            }
            if(raiseSlides.isDown()){
                slidesTargetPosition += 50;
            }
            if(lowerSlides.isDown()){
                slidesTargetPosition -= 50;
            }

            //SLIDES
            if (Math.abs(turret.turretMotor.getCurrentPosition())  >= turretTargetPosition - 10
                    && Math.abs(slides.slidesLeft.getCurrentPosition()) <= turretTargetPosition + 10){
               turret.turretMotor.setPower(0);
            }
            else if(Math.abs(slides.slidesLeft.getCurrentPosition())  <= turretTargetPosition - 10){
                turret.turretMotor.setPower(0.2);
            }
            else if(Math.abs(slides.slidesLeft.getCurrentPosition())  >= slidesTargetPosition + 10){
                turret.turretMotor.setPower(-0.2);
            }
            if(turretLeft.isDown()){
                turretTargetPosition += 50;
            }
            if(turretRight.isDown()){
                turretTargetPosition -= 50;
            }

            //GROUND INTAKE
            if (activeGround.isDown()) {
                groundIntake.setState(GroundIntake.State.INTAKING);
            }
            else{
                groundIntake.setState(GroundIntake.State.OFF);
            }

            //TELEMETRY
            telemetry.addData("V4B State: ",fourbar.getState());
            telemetry.addData("1 Ticks: ", robot.slides.slidesLeft.getCurrentPosition());
            telemetry.addData("2 Ticks: ", robot.slides.slidesLeft.getCurrentPosition());
            telemetry.update();

            if (reset.wasJustPressed()) {
                //       turret.setState(Turret.State.RESET);
                //     slides.setState(Slides.State.INTAKE);
                fourbar.setState(vfourb.State.PRIMED);
                //   intake.setState(Intake.State.OFF);
            }
        }
    }
}