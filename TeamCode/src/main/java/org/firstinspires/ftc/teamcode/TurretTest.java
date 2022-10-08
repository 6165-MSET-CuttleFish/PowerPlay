package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Turret Test Module")
@Config
public class TurretTest extends LinearOpMode
{
    Turret turret;
    ElapsedTime time=new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double TARGETPOSITION = 0;
    public static double CurrentPower = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {

        turret =new Turret(hardwareMap);
        turret.update();
        turret.turretMotor.setPower(0);
        telemetry.addData("Power", turret.turretMotor.getPower());
        telemetry.addData("STATE", turret.getState());
        telemetry.update();
        waitForStart();
        time.reset();
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive()) {
            telemetry.addData("Pos",turret.turretMotor.getCurrentPosition());
            telemetry.addData("STATE", turret.getState());
            if (turret.turretMotor.getCurrentPosition() == TARGETPOSITION) {
                turret.setState(Turret.State.IDLE);
            }else if (turret.turretMotor.getCurrentPosition() > TARGETPOSITION) {
                turret.setState(Turret.State.LEFT_ROTATE);
            }
            else if (turret.turretMotor.getCurrentPosition() < TARGETPOSITION) {
                turret.setState(Turret.State.RIGHT_ROTATE);
            }
            telemetry.update();
        }
    }
}
