package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        ninja = new ButtonReader(gm1, GamepadKeys.Button.LEFT_BUMPER);
        straight = new ButtonReader(gm1, GamepadKeys.Button.RIGHT_BUMPER);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while(!isStopRequested()){
            
            if (robot.getState() == Robot.driveState.NORMAL)
                normal();
            if (robot.getState() == Robot.driveState.STRAIGHT)
                straight();
            if (robot.getState() == Robot.driveState.NINJA)
                ninja();

            if (ninja.isDown()) {
                switch (robot.state){
                    case NORMAL:
                    case STRAIGHT:
                        robot.setState(Robot.driveState.NINJA);
                        break;
                    case NINJA:
                        robot.setState(Robot.driveState.NORMAL);
                        break;
                }
            }

            else if(straight.isDown()){
                switch (robot.state){
                    case NORMAL:
                    case NINJA:
                        robot.setState(Robot.driveState.STRAIGHT);
                        break;
                    case STRAIGHT:
                        robot.state = Robot.driveState.NORMAL;
                        break;
                }
            }
            robot.update();
            telemetry.addData("State", robot.getState());
            telemetry.update();
        }
    }
    private void normal(){
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y, //Math.abs(-gamepad1.left_stick_y) < 0.1 ? 0 :
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
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
                        -gamepad1.left_stick_y * 0.5,
                        gamepad1.left_stick_x * 0.5,
                        -gamepad1.right_stick_x * 0.5
                )
        );
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
