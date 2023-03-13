package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.deposit.Claw;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.util.BackgroundCR;
import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp
public class presflex extends LinearOpMode {
    Robot robot;
    Slides slides;
    Deposit deposit;
    GroundIntake groundIntake;
    Claw claw;
    Turret turret;
    BackgroundCR hardware;
    TelemetryPacket packet;
    GamepadEx primary;
    ToggleButtonReader extend;
    @Override

    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        primary = new GamepadEx(gamepad1);
        slides = robot.slides;
        claw = robot.claw;
        deposit = robot.deposit;
        groundIntake = robot.groundIntake;
        turret = robot.turret;
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Context.hallEffectEnabled=false;
        extend = new ToggleButtonReader(primary, GamepadKeys.Button.A);
        robot.turretCamera.stopStreaming();

        waitForStart();
        while (opModeIsActive()){
            robot.update();
            extend.readValue();
            if(extend.wasJustPressed() && deposit.getAngState() == Deposit.AngleState.VECTORING){
                turret.setState(Turret.State.ZERO);

                deposit.setExtension(Deposit.ExtensionState.RETRACT);
                deposit.setAngle(Deposit.AngleState.INTAKE);
                claw.setState(Claw.State.CLOSE);
                slides.setState(Slides.State.BOTTOM);
            }
            else if(extend.wasJustPressed() && deposit.getAngState() == Deposit.AngleState.INTAKE){
                slides.setState(Slides.State.HIGH);
                deposit.setExtension(Deposit.ExtensionState.EXTEND);
                deposit.setAngle(Deposit.AngleState.VECTORING);
                claw.setState(Claw.State.OPEN_WIDE);
                turret.setState(Turret.State.BACK);
            }
        }
    }
}
