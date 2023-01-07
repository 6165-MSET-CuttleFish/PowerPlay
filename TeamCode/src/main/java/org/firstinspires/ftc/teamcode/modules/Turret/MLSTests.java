package org.firstinspires.ftc.teamcode.modules.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;


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
