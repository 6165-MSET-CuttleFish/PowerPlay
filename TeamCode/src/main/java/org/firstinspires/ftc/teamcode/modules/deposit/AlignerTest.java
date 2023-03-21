package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Context;
@TeleOp
public class AlignerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Context.hardwareMap=hardwareMap;
        Claw claw = new Claw(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.cross)
                claw.setPoleState(Claw.Pole.DOWN);
            if (gamepad1.triangle)
                claw.setPoleState(Claw.Pole.UP);
            telemetry.addData("state: ", claw.getPoleState());
            telemetry.update();
        }
    }
}
