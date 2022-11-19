package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TurretZeroTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);
        double start = turret.turretMotor.getCurrentPosition();
        turret.turretMotor.setTargetPositionTolerance(5);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        double randomPos = start;
        int counter = 0;
        while(opModeIsActive()){

            if(counter%5==0){
                randomPos = 0;
            }
            else if(randomPos>=0){
                randomPos = - Math.random() * 367.0;
            }
            else randomPos = Math.random() * 367.0;
            counter++;
            int targetPos = (int) randomPos;
            turret.turretMotor.setTargetPosition(targetPos);
            while(turret.turretMotor.getCurrentPosition()>turret.turretMotor.getTargetPosition()+5 ||
                    turret.turretMotor.getCurrentPosition() < turret.turretMotor.getTargetPosition()-5){
                turret.turretMotor.setPower(0.25);
            }
           
        }
    }
}
