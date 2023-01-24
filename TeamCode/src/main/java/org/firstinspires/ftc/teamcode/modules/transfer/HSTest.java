package org.firstinspires.ftc.teamcode.modules.transfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "HS Tester")
@Config
public class HSTest extends OpMode {
    private Servo lA,lE,rA,rE;
    public static double anglePos=0; //0 is horizontal
    public static double extPos=0; //currently having issues :)
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        lA = hardwareMap.get(Servo.class, "lAng");
        rA = hardwareMap.get(Servo.class, "rAng");
        lE = hardwareMap.get(Servo.class, "lExt");
        rE = hardwareMap.get(Servo.class, "rExt");
    }

    @Override
    public void loop() {
        lA.setPosition(anglePos);
        rA.setPosition(1-anglePos);
        lE.setPosition(extPos);
        rE.setPosition(1-extPos);
        telemetry.addData("Current Servo Position Angle", lA.getPosition());
        telemetry.addData("Current Servo Position Extension", lE.getPosition());
        telemetry.update();
    }
}
