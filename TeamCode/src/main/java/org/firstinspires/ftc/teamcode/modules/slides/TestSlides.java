package org.firstinspires.ftc.teamcode.modules.slides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            slidesLeft.setPower(gamepad1.left_stick_y);
            slidesRight.setPower(gamepad1.right_stick_y);

            if (gamepad1.left_bumper) {
                slide.setState(Slides.State.HIGH);
            }
            if (gamepad1.right_bumper) {
                slide.setState(Slides.State.LOW);
            }
            telemetry.addData("Left Ticks: ", slidesLeft.getCurrentPosition());
            telemetry.addData("Right Ticks: ", slidesRight.getCurrentPosition());
            telemetry.addData("1 Power: ", slidesLeft.getPower());
            telemetry.addData("2 Power: ", slidesRight.getPower());
            telemetry.addData("Target Pos:", slidesRight.getTargetPosition());
            telemetry.addData("InchesL: ", slide.slidesLeft.getCurrentPosition());
            telemetry.addData("InchesR: ", slide.slidesRight.getCurrentPosition());
            telemetry.addData("currentR: ", slide.slidesRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("currentL: ", slide.slidesLeft.getCurrent(CurrentUnit.AMPS));

            telemetry.update();

        }




    }
}
