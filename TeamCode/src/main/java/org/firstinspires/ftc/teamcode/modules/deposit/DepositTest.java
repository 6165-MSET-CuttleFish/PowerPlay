package org.firstinspires.ftc.teamcode.modules.deposit;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp
public class DepositTest extends LinearOpMode {
    Servo leftExtension;
    Servo leftAngular;
    Servo rightExtension;
    Servo rightAngular;
    GamepadEx primary;
    double timer = 0;

    KeyReader[] keyReaders;
    ButtonReader depo;

    @Override
    public void runOpMode() throws InterruptedException {
        Context.hardwareMap=hardwareMap;
        Robot robot = new Robot(this);
        robot.claw.setState(Claw.State.CLOSE);
        sleep(2000);
        robot.slides.setState(Slides.State.CYCLE_HIGH);
        sleep(1000);
        robot.turret.setState(Turret.State.LEFT_SIDE_HIGH);

        waitForStart();

        primary = new GamepadEx(gamepad1);
        keyReaders = new KeyReader[]{
                depo = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN)
        };
        /*
        while (opModeIsActive())
        {

            /*
            if (gamepad1.a)
                robot.deposit.setExtension(Deposit.ExtensionState.EXTEND);
            if (gamepad1.b)
                robot.deposit.setExtension(Deposit.ExtensionState.FOURTH);
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
                robot.claw.setPoleState(Claw.Pole.DOWN);*/
            sleep(2000);
                robot.deposit.setAngle(Deposit.AngleState.VECTORING);
                //robot.claw.setPoleState(Claw.Pole.DEPOSIT);
                robot.deposit.setExtension(Deposit.ExtensionState.EXTEND);

                    robot.turret.setState(Turret.State.AUTOALIGN);
                    timer = System.currentTimeMillis();
                    while (System.currentTimeMillis() - 325 < timer){
                        robot.turret.update();
                        robot.update();
                    }
                robot.deposit.setAngle(Deposit.AngleState.INTAKE);
                timer = System.currentTimeMillis();
                while (System.currentTimeMillis() - 75< timer) {
                   robot.update();
                }

                //deposit.setState(Deposit.AngleState.INTAKE);
                robot.claw.setState(Claw.State.OPEN);
                timer = System.currentTimeMillis();
                while (System.currentTimeMillis() - 125< timer) {
                    robot.update();
                }



                robot.turret.setState(Turret.State.IDLE);

                //robot.claw.setPoleState(Claw.Pole.DOWN);
                robot.deposit.setExtension(Deposit.ExtensionState.RETRACT);
                robot.deposit.setAngle(Deposit.AngleState.INTAKE);
                //claw.setPoleState(Claw.Pole.UP);
                timer = System.currentTimeMillis();
                while (System.currentTimeMillis() - 325 < timer) {
                    robot.update();
                }
            //}
            telemetry.addData("Ext State: ", robot.deposit.getExtState());
            telemetry.addData("Ang State: ", robot.deposit.getAngState());
            telemetry.addData("clawState: ",robot.claw.getState());
            telemetry.addData("leftEXT: ", robot.deposit.leftExtension.getPosition());
            telemetry.addData("rightEXT: ", robot.deposit.rightExtension.getPosition());
            telemetry.addData("wrist: ", robot.deposit.wrist.getPosition());
            telemetry.update();


    }
}
