package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.transfer.Intake;

@TeleOp
public class WristTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                deposit.setAngle(Deposit.AngleState.INTAKE);
            if (gamepad1.b)
                deposit.setAngle(Deposit.AngleState.VECTORING);
            telemetry.addData("state: ", deposit.getAngState());
        }
    }
}
