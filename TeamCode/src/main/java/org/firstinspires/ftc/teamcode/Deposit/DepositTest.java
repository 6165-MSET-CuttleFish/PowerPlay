package org.firstinspires.ftc.teamcode.Deposit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class DepositTest extends LinearOpMode {
    Servo deposit1;
    Servo deposit2;
    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                deposit.setState(Deposit.State.EXTEND);
            if (gamepad1.b)
                deposit.setState(Deposit.State.MIDDLE);
            if (gamepad1.x)
                deposit.setState(Deposit.State.RETRACT);

            telemetry.addData("State: ", deposit.getState());
            telemetry.addData("Position1: ", deposit.deposit1.getPosition());
            telemetry.addData("Position2: ", deposit.deposit1.getPosition());
            telemetry.update();

        }




    }
}
