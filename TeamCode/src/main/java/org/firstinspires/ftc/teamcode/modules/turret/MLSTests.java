package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name = "MLS")
@Config
public class MLSTests extends OpMode {
    private AnalogInput hallEffect;
    public double tolerance=1;
    public double prev=5.0;
    double targetPos, prevReset=0;
    public Encoder encoder;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        hallEffect = hardwareMap.get(AnalogInput.class, "hallEffect");
    }

    @Override
    public void loop() {
        if(hallEffect.getVoltage()<prev){
            telemetry.addData("Fall"," Detected");
        }
        telemetry.addData("Hall Effect voltage: ", (hallEffect.getVoltage()));
        telemetry.addData("Hall Effect Activated: ", (hallEffect.getVoltage()<tolerance));
        telemetry.update();
        prev=hallEffect.getVoltage()-0.1;

    }
}
