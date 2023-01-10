package org.firstinspires.ftc.teamcode.modules.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class SimpleTurretTest extends LinearOpMode {
    DcMotorEx turretMotor;
    Encoder encoder;
    @Override
    public void runOpMode() throws InterruptedException {
        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            turretMotor.setPower(gamepad1.right_stick_x);
            telemetry.addData("ticks: ", encoder.getCurrentPosition());
            telemetry.update();
        }

        }
    }