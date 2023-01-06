package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "MLS")
public class MLSTests extends OpMode {
    private TouchSensor magnetic;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        magnetic = hardwareMap.get(TouchSensor.class, "MLS");
    }

    @Override
    public void loop() {
        telemetry.addData("MLS Activated: ", magnetic.isPressed());
        telemetry.update();
    }
}
