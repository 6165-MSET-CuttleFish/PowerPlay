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

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.Transfer.Intake;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;
import org.firstinspires.ftc.teamcode.Turret.Turret;

@TeleOp
public class practiceDriverCode extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slide;
    vfourb fourbar;
    GroundIntake groundIntake;
     Turret turret;
    GamepadEx primary;
    GamepadEx secondary;
    KeyReader[] keyReaders;
    TriggerReader intakeButton, deposit;
    ButtonReader liftHigh, liftMedium, liftLow, junction, align ,reset,raiseLift,lowerLift,ninjaMode,liftBot;
    ToggleButtonReader activeGround;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        slide = robot.slides;
        intake = robot.intake;
        fourbar = robot.fourbar;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        keyReaders = new KeyReader[]{
                intakeButton = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                ninjaMode = new ButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                liftHigh = new ButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                liftMedium = new ButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                liftBot = new ButtonReader(primary, GamepadKeys.Button.DPAD_RIGHT),
                liftLow = new ButtonReader(primary, GamepadKeys.Button.DPAD_LEFT),
                junction = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                deposit = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),
                align = new ButtonReader(primary, GamepadKeys.Button.X),
                reset = new ButtonReader(primary, GamepadKeys.Button.B),
                raiseLift = new ButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                lowerLift = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                activeGround = new ToggleButtonReader(primary, GamepadKeys.Button.A),
        };
        waitForStart();

  //      slide.setState(Slides.State.BOTTOM);
        fourbar.setState(vfourb.State.PRIMED);
        while (opModeIsActive()) {

            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }
        //    turret.setState(Turret.State.IDLE);
          //  turret.update();
      //      slide.update();
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
                    fourbar.setState(vfourb.State.DEPOSIT_POSITION);
                }
                //ground intake auto
             /*        if(groundIntake.gSensor()) {
                intake.setState(Intake.State.INTAKING);
                fourbar.setState(vfourb.State.INTAKE_POSITION);
            }*/
            //cant go to deposit position as slides go up b/c robot can't go high enough
                if(liftMedium.isDown()){
                    slide.up();
                }
                else if(junction.isDown()){
                    slide.mid();
                }
                else if(liftBot.isDown()){
                    slide.low();
                }
                else if(liftLow.isDown()){
                    slide.down();
                }
                //this is not a good impl
                if(align.isDown()){
//                    turret.zero();
                }
                /*
*/
                if (activeGround.wasJustPressed()) {
                    if(groundIntake.getState() == GroundIntake.State.INTAKING){
                        groundIntake.setState(GroundIntake.State.OFF);
                    }
                    else {
                        groundIntake.setState(GroundIntake.State.INTAKING);
                    }
                }
telemetry.update();

                if (reset.wasJustPressed()) {
                //    slide.setState(Slides.State.BOTTOM);
                    fourbar.setState(vfourb.State.PRIMED);
                }
                telemetry.addData("jgo",turret.turretMotor.getTargetPositionTolerance());
                telemetry.addData("jgo1",turret.turretMotor.getTargetPosition());
                telemetry.addData("jgo2",turret.turretMotor.getCurrentPosition());
                telemetry.addData("jgo3",turret.turretMotor.getVelocity());
            }
        }
    }
