package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

@TeleOp
public class limitTest extends LinearOpMode {
    AnalogInput touch;
    @Override
    public void runOpMode() throws InterruptedException {
        touch = hardwareMap.get(AnalogInput.class, "limit");
        waitForStart();
        while(opModeIsActive()){
            double sensor = touch.getVoltage();
            telemetry.addData("alksdjfask", sensor);
            telemetry.update();
        }
    }
}
