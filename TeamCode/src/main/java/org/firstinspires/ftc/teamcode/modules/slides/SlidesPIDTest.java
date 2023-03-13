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
        slides = new Slides();
        GamepadEx gm1 = new GamepadEx(gamepad1);
        bottom = new ButtonReader(gm1, GamepadKeys.Button.A);
        middle = new ButtonReader(gm1, GamepadKeys.Button.B);
        high = new ButtonReader(gm1, GamepadKeys.Button.Y);

        waitForStart();
        while(opModeIsActive()){
//            slides.update();
            if(gamepad1.a){
                pidController.setTargetPosition(100);
            }
            else if(gamepad1.b){
                pidController.setTargetPosition(1180);
            }
            else if(gamepad1.y){
                pidController.setTargetPosition(2300);
            }
            double output = pidController.update(slides.slidesRight.getCurrentPosition());
            slides.slidesRight.setPower(output);
            slides.slidesLeft.setPower(output);

            telemetry.addData("targetPos: ", pidController.getTargetPosition());
            telemetry.addData("output power: ", output);
            telemetry.addData("currentPos: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("currentVelo: ", slides.slidesLeft.getVelocity());
            telemetry.addData("currentPower: ", slides.slidesLeft.getPower());
            telemetry.addData("currentPos: ", slides.slidesRight.getCurrentPosition());
            telemetry.addData("currentVelo: ", slides.slidesRight.getVelocity());
            telemetry.addData("currentPower: ", slides.slidesRight.getPower());

            telemetry.update();
        }

    }
}
