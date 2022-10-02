package org.firstinspires.ftc.teamcode.Slides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "slides")
public class TestSlides extends LinearOpMode {
    Slides slide;
    ElapsedTime time;
    @Override
    public void runOpMode() throws InterruptedException {

        slide = new Slides(hardwareMap);

        waitForStart();

        slide.update();


    }
}
