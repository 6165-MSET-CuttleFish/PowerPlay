package org.firstinspires.ftc.teamcode.modules.slides;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.BPIDFController;

@TeleOp
public class SlidesPIDTest extends LinearOpMode {
    Slides slides;
    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(0.7,0,0.01);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    private PIDController pidController = new PIDController(0,0,0);

    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap);


        waitForStart();
        while(opModeIsActive()){
          //  slides.update();
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
