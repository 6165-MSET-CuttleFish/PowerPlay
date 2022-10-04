package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    public ToggleButtonReader cycle;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.setState(Robot.driveState.normal);
        cycle = new ToggleButtonReader(gm1, GamepadKeys.Button.LEFT_BUMPER);

        waitForStart();
        while(opModeIsActive()){
            if(robot.getState()== Robot.driveState.normal) normal();
            else if(robot.getState()== Robot.driveState.straight) straight();
            else if(robot.getState()== Robot.driveState.ninja) ninja();
            if(cycle.wasJustPressed()){
                switch (robot.getState()){
                    case normal:
                        robot.setState(Robot.driveState.ninja);
                        break;
                    case ninja:
                        robot.setState(Robot.driveState.straight);
                        break;
                    case straight:
                        robot.setState(Robot.driveState.normal);
                        break;
                }
            }

        }
    }
    private void normal(){
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gm1.getLeftY(),
                        -gm1.getLeftX(),
                        -gm1.getRightX()
                )
        );
    }
    private void straight(){
        if(Math.abs(gm1.getLeftY()) > Math.abs(gm1.getLeftX())){
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gm1.getLeftY(),
                            0,
                            -gm1.getRightX()
                    )
            );
        }
        else{
            robot.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            -gm1.getLeftX(),
                            -gm1.getRightX()
                    )
            );
        }
    }
    private void ninja(){
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gm1.getLeftY()*0.2,
                        -gm1.getLeftX()*0.2,
                        -gm1.getRightX()*0.2
                )
        );
    }
}
