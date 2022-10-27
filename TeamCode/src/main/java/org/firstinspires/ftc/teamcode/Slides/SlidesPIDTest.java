package org.firstinspires.ftc.teamcode.Slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SlidesPIDTest extends LinearOpMode {
    Slides slides;
    public static double targetPosition = 16;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap);
        slides.slidesRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slides.slidesLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slides.slidesLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slides.slidesRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        double output = 0;
        while(opModeIsActive()){
            if (gamepad1.y) {
                targetPosition = Slides.HIGH;
            } else if (gamepad1.a) {
                targetPosition = Slides.MID;
            }
            else if (gamepad1.b) {
                targetPosition = Slides.LOW;
            }
            output = Slides.pidController.calculate(-Slides.inchesToTicks(targetPosition), -slides.slidesLeft.getCurrentPosition());
            // assign motor the PID output
            if (-Slides.inchesToTicks(targetPosition) < -slides.slidesLeft.getCurrentPosition())
                output *= 0.8;

            slides.slidesLeft.setPower(output);
            slides.slidesRight.setPower(output);

//            if (Slides.inchesToTicks(targetPosition) == -slides.slidesLeft.getCurrentPosition()) {
//                slides.slidesLeft.setPower(0);
//                slides.slidesRight.setPower(0);
//            }

            telemetry.addData("targetPos: ", targetPosition);
            telemetry.addData("currentPos: ", Slides.ticksToInches(slides.slidesLeft.getCurrentPosition()));

            telemetry.update();
        }

    }
}
