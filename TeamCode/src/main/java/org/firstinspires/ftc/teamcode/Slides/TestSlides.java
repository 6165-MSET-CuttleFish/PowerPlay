package org.firstinspires.ftc.teamcode.Slides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "slides")
public class TestSlides extends LinearOpMode {
    Slides slide;
    DcMotorEx slidesLeft;
    DcMotorEx slidesRight;
    @Override
    public void runOpMode() throws InterruptedException {

        slide = new Slides(hardwareMap);
        slidesLeft = hardwareMap.get(DcMotorEx.class, "s1");
        slidesRight = hardwareMap.get(DcMotorEx.class, "s2");

        waitForStart();

        while (opModeIsActive()) {
            slidesLeft.setPower(-gamepad1.left_stick_y);
            slidesRight.setPower(gamepad1.right_stick_y);

            if (gamepad1.left_bumper) {
                slidesLeft.setPower(0.2);
                slidesRight.setPower(0.2);
            }
            if (gamepad1.right_bumper) {
                slidesLeft.setPower(-0.2);
                slidesRight.setPower(-0.2);
            }
            telemetry.addData("Left Ticks: ", slidesLeft.getCurrentPosition());
            telemetry.addData("Right Ticks: ", slidesRight.getCurrentPosition());
            telemetry.addData("Power: ", slidesRight.getPower());
            telemetry.addData("InchesL: ", slide.ticksToInches(slidesLeft.getCurrentPosition()));
            telemetry.addData("InchesR: ", slide.ticksToInches(slidesRight.getCurrentPosition()));
            telemetry.addData("currentR: ", slide.ticksToInches(slidesRight.getCurrent(CurrentUnit.AMPS)));
            telemetry.addData("currentL: ", slide.ticksToInches(slidesLeft.getCurrent(CurrentUnit.AMPS)));

            telemetry.update();

        }




    }
}
