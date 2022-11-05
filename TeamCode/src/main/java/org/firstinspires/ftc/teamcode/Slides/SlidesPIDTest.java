package org.firstinspires.ftc.teamcode.Slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SlidesPIDTest extends LinearOpMode {
    Slides slides;
    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap);
     //   slides.slidesRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
       // slides.slidesLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        waitForStart();
        while(opModeIsActive()){
          //  slides.update();
            if (gamepad1.y) {
                slides.setState(Slides.State.HIGH);
            } else if (gamepad1.a) {
                slides.setState(Slides.State.MID);
            } else if (gamepad1.b) {
                slides.setState(Slides.State.LOW);
            } else if (gamepad1.x) {
                slides.setState(Slides.State.BOTTOM);
            }

            telemetry.addData("targetPos: ", slides.getState());
            telemetry.addData("currentPos: ", slides.slidesLeft.getCurrentPosition());
            telemetry.addData("currentVelo: ", slides.slidesLeft.getVelocity());
            telemetry.addData("currentVelo: ", slides.slidesLeft.getPower());

            telemetry.update();
        }

    }
}
