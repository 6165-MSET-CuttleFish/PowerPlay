package org.firstinspires.ftc.teamcode.modules.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotTemp;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;

@TeleOp
public class Turret2Test extends LinearOpMode
{
    Turret turret;
    Deposit deposit;
    RobotTemp robot;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot=new RobotTemp(this, true);
        turret=robot.turret;
        deposit = robot.deposit;
        timer=new ElapsedTime();

        while(!isStarted())
        {
            telemetry.addData("Encoder", turret.encoder.getCurrentPosition());
            telemetry.addData("Target Pos", turret.getTargetPos());
            telemetry.update();
        }
        waitForStart();




        //turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //turret.setState(Turret2.State.LEFT);
        while(opModeIsActive())
        {
            robot.slides.setState(Slides.State.MID);
            if(gamepad1.a)
            {
                turret.setState(Turret.State.BACK);
            }
            else if(gamepad1.b)
            {
                turret.setState(Turret.State.RIGHT_SIDE_HIGH);
            }
            else if(gamepad1.y)
            {
                turret.setState(Turret.State.ZERO);
            }
            if(gamepad1.dpad_up){
                deposit.setExtension(Deposit.ExtensionState.EXTEND);
            }
            else if(gamepad1.dpad_down) deposit.setExtension(Deposit.ExtensionState.RETRACT);
            //turret.update();
            telemetry.addData("State", turret.getState());
            telemetry.addData("power", turret.turretMotor.getPower());
            telemetry.addData("encoder", turret.encoder.getCurrentPosition());
            telemetry.addData("Target Pos", turret.getTargetPos());
            //telemetry.addData("pid", turret.pidMotorOil);
            telemetry.update();
        }
    }
}
