package org.firstinspires.ftc.teamcode.ground;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "GI Tester")
@Config
public class GITest extends OpMode {
    private DcMotor groundIntake;
    public static double power=1; //1 is intake, -1 is outtake
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        groundIntake = hardwareMap.get(DcMotor.class, "gIntake");
        groundIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        groundIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        groundIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        groundIntake.setPower(power);
        telemetry.addData("GI", groundIntake.getPower());
        telemetry.update();
    }
}
