package org.firstinspires.ftc.teamcode.modules.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IncrementalTurretTest extends LinearOpMode {
    Turret turret;
    @Override
    public void runOpMode() {
        turret = new Turret(hardwareMap, false);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                turret.setState(Turret.State.BACK);
            if (gamepad1.b)
                turret.setState(Turret.State.RIGHT);
            if (gamepad1.x)
                turret.setState(Turret.State.LEFT);
            if (gamepad1.y)
                turret.setState(Turret.State.ZERO);

            telemetry.addData("State: ", turret.getState());
            telemetry.addData("ticks: ", turret.encoder.getCurrentPosition());
            telemetry.addData("target position: ", turret.getTargetPos());
            telemetry.update();
        }
    }
}
