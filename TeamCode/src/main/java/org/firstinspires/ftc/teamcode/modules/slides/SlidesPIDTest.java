package org.firstinspires.ftc.teamcode.modules.slides;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.BPIDFController;
import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp
@Config
public class SlidesPIDTest extends LinearOpMode {
    Slides slides;
     public ButtonReader bottom, middle, high;
    public static double p = 0.02, i = 0, d = 0.0002;
    public static double kV = 0, kA = 0, kStatic = 0;
    private BPIDFController pidController = new BPIDFController(new PIDCoefficients(p,i,d), kV, kA, kStatic);

    public void runOpMode() throws InterruptedException {
        Context.hardwareMap=hardwareMap;
        Robot robot = new Robot(this);
        slides = robot.slides;
        GamepadEx gm1 = new GamepadEx(gamepad1);
        bottom = new ButtonReader(gm1, GamepadKeys.Button.A);
        middle = new ButtonReader(gm1, GamepadKeys.Button.B);
        high = new ButtonReader(gm1, GamepadKeys.Button.Y);

        waitForStart();
        while(opModeIsActive()){
//            slides.update();
            if(gamepad1.a){
                slides.setState(Slides.State.BOTTOM);
            }
            else if(gamepad1.b){
                slides.setState(Slides.State.MID);
            }
            else if(gamepad1.y){
                slides.setState(Slides.State.HIGH);
            }

            telemetry.addData("targetPos: ", slides.pidController.getTargetPosition());
            telemetry.addData("LEFT currentPos: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("LEFT currentPower: ", slides.slidesLeft.getPower());
            telemetry.addData("RIGHT currentPos: ", slides.slidesRight.getCurrentPosition());
            telemetry.addData("RIGHT currentPower: ", slides.slidesRight.getPower());
            telemetry.update();
        }

    }
}
