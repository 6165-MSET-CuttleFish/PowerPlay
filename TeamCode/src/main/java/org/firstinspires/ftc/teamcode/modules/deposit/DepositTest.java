package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
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
        Robot robot = new Robot(this);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                robot.deposit.setExtension(Deposit.ExtensionState.EXTEND);
            if (gamepad1.b)
                robot.deposit.setExtension(Deposit.ExtensionState.RETRACT);
            if (gamepad1.x)
                robot.deposit.setAngle(Deposit.AngleState.INTAKE);
            if (gamepad1.y)
                robot.deposit.setAngle(Deposit.AngleState.VECTORING);
            if(gamepad1.right_bumper)
                robot.claw.setState(Claw.State.OPEN);
            if(gamepad1.left_bumper)
                robot.claw.setState(Claw.State.CLOSE);
            if(gamepad1.dpad_up)
                robot.claw.setPoleState(Claw.Pole.UP);
            if(gamepad1.dpad_down)
                robot.claw.setPoleState(Claw.Pole.DOWN);

            telemetry.addData("Ext State: ", robot.deposit.getExtState());
            telemetry.addData("Ang State: ", robot.deposit.getAngState());
            telemetry.addData("clawState: ",robot.claw.getState());
            telemetry.addData("leftEXT: ", robot.deposit.leftExtension.getPosition());
            telemetry.addData("rightEXT: ", robot.deposit.rightExtension.getPosition());
            telemetry.addData("wrist: ", robot.deposit.wrist.getPosition());
            telemetry.update();

        }
    }
}
