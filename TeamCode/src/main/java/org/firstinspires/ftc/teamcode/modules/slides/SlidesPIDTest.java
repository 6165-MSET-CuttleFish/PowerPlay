package org.firstinspires.ftc.teamcode.modules.slides;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.BPIDFController;

@TeleOp
public class SlidesPIDTest extends LinearOpMode {
    Slides slides;
     public ButtonReader bottom, middle, high;
    public static double p,i,d;
    private PIDController pidController = new PIDController(p,i,d);

    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap);
        GamepadEx gm1 = new GamepadEx(gamepad1);
        bottom = new ButtonReader(gm1, GamepadKeys.Button.A);
        middle = new ButtonReader(gm1, GamepadKeys.Button.B);
        high = new ButtonReader(gm1, GamepadKeys.Button.Y);

        waitForStart();
        while(opModeIsActive()){
          //  slides.update();
            if(bottom.wasJustPressed()){
                slides.slidesLeft.setTargetPosition(0);
                slides.slidesRight.setTargetPosition(0);
            }
            else if(middle.wasJustPressed()){
                slides.slidesLeft.setTargetPosition(1180);
                slides.slidesRight.setTargetPosition(1180);
            }
            else if(high.wasJustPressed()){
                slides.slidesLeft.setTargetPosition(2050);
                slides.slidesRight.setTargetPosition(2050);
            }
            double output = pidController.calculate(slides.slidesRight.getCurrentPosition());
            slides.setPowerManual(output);

            telemetry.addData("targetPos: ", slides.getState());
            telemetry.addData("currentPos: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("currentVelo: ", slides.slidesLeft.getVelocity());
            telemetry.addData("currentPower: ", slides.slidesLeft.getPower());
            telemetry.addData("PID: ", slides.slidesLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Limit Switch: ", slides.slidesLimitSwitch.getState());


            telemetry.update();
        }

    }
}
