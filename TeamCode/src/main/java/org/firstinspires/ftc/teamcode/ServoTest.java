package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo Tester")
@Config
public class ServoTest extends OpMode {
    private Servo servo;
    public static double position=0;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "testServo");
    }

    @Override
    public void loop() {
        if(servo.getPosition()!=position) {
            servo.setPosition(position);
        }
        telemetry.addData("Target Servo Position", position);
        telemetry.addData("Current Servo Position", servo.getPosition());
        telemetry.update();
    }
}
