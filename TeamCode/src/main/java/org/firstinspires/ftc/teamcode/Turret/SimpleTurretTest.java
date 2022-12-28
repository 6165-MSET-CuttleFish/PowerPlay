package org.firstinspires.ftc.teamcode.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Transfer.vfourb;

public class SimpleTurretTest extends LinearOpMode {
    Turret turret;
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        turret = robot.turret;
        boolean turretStop = false;
        while(!isStarted()) {
            telemetry.addData("encoder", turret.encoder.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                turret.setState(Turret.State.RIGHT);
            }
            if(gamepad1.b) {
                turret.setState(Turret.State.LEFT);
            }
            if(gamepad1.y) {
                turret.setState(Turret.State.ZERO);
            }

            if (Math.abs(gamepad2.right_stick_x) > 0) {
                turret.setState(Turret.State.MANUAL);
                turret.turretMotor.setPower(gamepad2.right_stick_x);
                turretStop = true;
            }
            if (turretStop && gamepad2.right_stick_x == 0) {
                turret.setState(Turret.State.MANUAL);
                turret.turretMotor.setPower(0);
                turretStop = false;
            }
            //turret.update();
            telemetry.addData("State", turret.getState());
            telemetry.addData("power", turret.motorOil);
            telemetry.addData("encoder", turret.encoder.getCurrentPosition());
            //telemetry.addData("pid", turret.pidMotorOil);
            telemetry.update();
        }
    }
}
