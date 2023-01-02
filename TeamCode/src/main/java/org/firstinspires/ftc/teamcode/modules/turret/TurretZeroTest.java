package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.transfer.vfourb;

@Autonomous
@Config
public class TurretZeroTest extends LinearOpMode {
    public static int target=0;


    public void runOpMode() throws InterruptedException {
        TurretOld turret = new TurretOld(hardwareMap);
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double start = turret.turretMotor.getCurrentPosition();
        turret.turretMotor.setTargetPositionTolerance(5);
        turret.turretMotor.setTargetPosition(target);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vfourb fourbar = new vfourb(hardwareMap);
        fourbar.setState(vfourb.State.PRIMED);
        waitForStart();
        double randomPos = start;
        int counter = 0;
        while(opModeIsActive()){

            //turret.turretMotor.setTargetPosition();
            turret.turretMotor.setTargetPosition(target);
            turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.turretMotor.setPower(0.5);
            telemetry.addData("Current Position", turret.turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
