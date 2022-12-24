package org.firstinspires.ftc.teamcode.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "MLS")
@Config
public class MLSTests extends OpMode {
    private TouchSensor magnetic;
    private AnalogInput hallEffect;
    public double tolerance=1;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        hallEffect = hardwareMap.get(AnalogInput.class, "hallEffect");
        magnetic = hardwareMap.get(TouchSensor.class, "MLS");
    }

    @Override
    public void loop() {
        telemetry.addData("MLS Activated: ", magnetic.isPressed());
        telemetry.addData("Hall Effect Voltage: ", hallEffect.getVoltage());
        telemetry.addData("Hall Effect Activated: ", (hallEffect.getVoltage()<tolerance));
        telemetry.update();
    }
}
