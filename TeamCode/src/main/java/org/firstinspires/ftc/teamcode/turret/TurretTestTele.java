package org.firstinspires.ftc.teamcode.turret;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TurretTestTele extends LinearOpMode {
    DcMotor turret;
    double position;

    @Override
    public void runOpMode() throws InterruptedException {
        turret= hardwareMap.get(DcMotor.class, "hturret");
        while (opModeIsActive()){
            turret.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            position=turret.getCurrentPosition();
            telemetry.addData("Current Position", position);
            telemetry.update();

        }
    }
}
