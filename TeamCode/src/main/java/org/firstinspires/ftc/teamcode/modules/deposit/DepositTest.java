package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp
public class DepositTest extends LinearOpMode {
    Servo leftExtension;
    Servo leftAngular;
    Servo rightExtension;
    Servo rightAngular;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Context.hardwareMap=hardwareMap;
        Deposit deposit = new Deposit();
        Claw claw = new Claw();
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
            if(gamepad1.right_bumper)
                claw.setState(Claw.State.OPEN);
            if(gamepad1.left_bumper)
                claw.setState(Claw.State.CLOSE);

            telemetry.addData("Ext State: ", deposit.getExtState());
            telemetry.addData("Ang State: ", deposit.getAngState());
            telemetry.addData("clawState: ",claw.getState());
            telemetry.addData("leftEXT: ", deposit.leftExtension.getPosition());
            telemetry.addData("rightEXT: ", deposit.rightExtension.getPosition());
            telemetry.addData("wrist: ", deposit.wrist.getPosition());
            telemetry.update();

        }
    }
}
