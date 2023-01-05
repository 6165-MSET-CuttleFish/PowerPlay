package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DepositTest extends LinearOpMode {
    Servo leftExtension;
    Servo leftAngular;
    Servo rightExtension;
    Servo rightAngular;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                deposit.setExtension(Deposit.ExtensionState.EXTEND);
            if (gamepad1.b)
                deposit.setExtension(Deposit.ExtensionState.RETRACT);
            if (gamepad1.x)
                deposit.setAngle(Deposit.AngleState.INTAKE);
            if (gamepad1.y)
                deposit.setAngle(Deposit.AngleState.VECTORING);

            telemetry.addData("Ext State: ", deposit.getExtState());
            telemetry.addData("Ang State: ", deposit.getAngState());
            telemetry.addData("leftEXT: ", deposit.leftExtension.getPosition());
            telemetry.addData("rightEXT: ", deposit.rightExtension.getPosition());
            telemetry.addData("leftANG: ", deposit.leftAngular.getPosition());
            telemetry.addData("rightANG: ", deposit.rightAngular.getPosition());
            telemetry.update();

        }
    }
}
