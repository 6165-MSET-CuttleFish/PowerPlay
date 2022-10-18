package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(group = "drive")
public class DrivetrainTest extends LinearOpMode {
    //public HardwareMap hardwareMap;
    public Robot robot;
    public GamepadEx gm1;
    public ButtonReader ninja, straight;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.setState(Robot.driveState.NORMAL);
        gm1 = new GamepadEx(gamepad1);
        ninja = new ToggleButtonReader(gm1, GamepadKeys.Button.LEFT_BUMPER);
        straight = new ToggleButtonReader(gm1, GamepadKeys.Button.RIGHT_BUMPER);
        boolean isNinja = false;
        boolean isStraight = false; //:)
        waitForStart();

        while(opModeIsActive()){
            if (!isNinja && !isStraight)
                normal();
            else if (!isNinja && isStraight)
                straight();
            else if (isNinja && !isStraight)
                ninja();
            else{
                combined();
            }

            if (ninja.wasJustPressed()) {
                if(isNinja) isNinja = false;
                else isNinja = true;
            }
            if (straight.wasJustPressed()) {
                if(isStraight) isStraight = false;
                else isStraight = true;
            }
            telemetry.addData("State", robot.getState());
            telemetry.update();
        }
    }
    private void normal(){
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.right_stick_x, //Math.abs(-gamepad1.left_stick_y) < 0.1 ? 0 :
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y
                )
        );
        //robot.setMotorPowers(1,1,1,1);
    }
    private void straight(){
        if(Math.abs(gm1.getLeftY()) > Math.abs(gm1.getLeftX())){
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            0,
                            -gamepad1.right_stick_x
                    )
            );
        }
        else{
            robot.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
        }
    }
    private void ninja(){
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.1,
                        gamepad1.left_stick_x * 0.1,
                        -gamepad1.right_stick_x * 0.1
                )
        );
        //robot.setMotorPowers(0.2,0.2,0.2,0.2);
    }
    private void combined(){
        if(Math.abs(gm1.getLeftY()) > Math.abs(gm1.getLeftX())){
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.1,
                            0,
                            -gamepad1.right_stick_x * 0.1
                    )
            );
        }
        else{
            robot.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            gamepad1.left_stick_x * 0.1,
                            -gamepad1.right_stick_x * 0.1
                    )
            );
        }
        //robot.setMotorPowers(0.2,0.2,0.2,0.2);
    }
}
