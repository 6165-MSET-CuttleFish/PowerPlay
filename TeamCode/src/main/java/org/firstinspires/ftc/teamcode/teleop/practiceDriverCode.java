package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.transfer.Intake;
import org.firstinspires.ftc.teamcode.transfer.vfourb;

@TeleOp
public class practiceDriverCode extends LinearOpMode {
    Robot robot;
    Intake intake;
    //Slides slides;
    Slides slide;
    vfourb fourbar;
    GroundIntake groundIntake;
    // Turret turret;
    GamepadEx primary;
    GamepadEx secondary;
    KeyReader[] keyReaders;
    TriggerReader intakeButton, deposit;
    ButtonReader liftHigh, liftMedium, liftLow, junction, align ,reset,raiseLift,lowerLift,ninjaMode;
    ToggleButtonReader activeGround;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = new Slides(hardwareMap);
        robot = new Robot(hardwareMap);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        intake = robot.intake;
      //  slides = robot.slides;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
      //  turret = robot.turret;
        keyReaders = new KeyReader[]{
                intakeButton = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                ninjaMode = new ButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                liftHigh = new ButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
              //  liftMedium = new ButtonReader(secondary, GamepadKeys.Button.DPAD_UP),
            //    liftLow = new ButtonReader(secondary, GamepadKeys.Button.DPAD_LEFT),
           //     junction = new ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN),
                deposit = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),
             //   align = new ButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER),
                reset = new ButtonReader(primary, GamepadKeys.Button.B),
                raiseLift = new ButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                lowerLift = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                activeGround = new ToggleButtonReader(primary, GamepadKeys.Button.A),
        };
        waitForStart();
    //    slides.setState(Slides.State.INTAKE);
        fourbar.setState(vfourb.State.PRIMED);
        while (opModeIsActive()) {
            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }
                if (ninjaMode.isDown()) {
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
                if (liftHigh.wasJustPressed()) {
                    slide.setState(Slides.State.HIGH);
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                    intake.setState(Intake.State.OFF);
                }
            if(groundIntake.gSensor()){
                intake.setState(Intake.State.INTAKING);
                fourbar.setState(vfourb.State.INTAKE_POSITION);
            }
                /*
                if (liftMedium.wasJustPressed()) {
                    slides.setState(Slides.State.MID);
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                }
                if (liftLow.wasJustPressed()) {
                    slides.setState(Slides.State.LOW);
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                }
                if (junction.wasJustPressed()) {
                    slides.setState(Slides.State.GROUND);
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                }
                if (align.wasJustPressed()) {
                    turret.setState(Turret.State.ALIGNING);
                }*/

                if (activeGround.wasJustPressed()) {
                    if(groundIntake.getState() == GroundIntake.State.INTAKING){
                        groundIntake.setState(GroundIntake.State.OFF);
                    }
                    else {
                        groundIntake.setState(GroundIntake.State.INTAKING);
                    }
                }
                telemetry.addData("V4B State: ",fourbar.getState());
            telemetry.addData("Left Ticks: ", slide.lpos());
            telemetry.addData("Right Ticks: ", slide.rpos());
telemetry.update();

                if (reset.wasJustPressed()) {
                    slide.setState(Slides.State.BOTTOM);
                    fourbar.setState(vfourb.State.PRIMED);
                }
            }
        }
    }
