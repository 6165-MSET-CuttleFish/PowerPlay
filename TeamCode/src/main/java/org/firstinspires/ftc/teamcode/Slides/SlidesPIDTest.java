package org.firstinspires.ftc.teamcode.Slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SlidesPIDTest extends LinearOpMode {
    Slides slides;
    public static double targetPosition;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap);
        waitForStart();
        slides.pidf.setSetPoint(targetPosition);
        while(!isStopRequested()){

            while (!slides.pidf.atSetPoint()) {
                double output = slides.pidf.calculate(
                        slides.slidesLeft.getCurrentPosition()  // the measured value
                );
                slides.slidesLeft.setVelocity(output);
                slides.slidesRight.setVelocity(output);
            }
            slides.slidesLeft.setPower(0.03);
            slides.slidesRight.setPower(0.03);
        }

    }
}
