package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Slides.Slides;
import org.firstinspires.ftc.teamcode.transfer.Intake;
import org.firstinspires.ftc.teamcode.transfer.vfourb;
import org.firstinspires.ftc.teamcode.turret.Turret;

@TeleOp
public class practiceDriverCode extends LinearOpMode {
    Robot robot;
    Intake intake;
    Slides slides;
    vfourb fourbar;
    Turret turret;
    GamepadEx primary;
    GamepadEx secondary;
    KeyReader[] keyReaders;
    TriggerReader intakeButton, ninjaMode;
    ButtonReader liftHigh, liftMedium, liftLow, deposit, intakeButton
    ToggleButtonReader activeGround, closeDeposit, farDeposit, crossDeposit, mediumDeposit;

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
