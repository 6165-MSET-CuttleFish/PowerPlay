package org.firstinspires.ftc.teamcode.modules.moduleUtil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.Transfer.Claw;
import org.firstinspires.ftc.teamcode.modules.Transfer.ExtensionAngle;
import org.firstinspires.ftc.teamcode.modules.Turret.Turret;
import org.firstinspires.ftc.teamcode.modules.ground.GroundIntake;

@Autonomous
public class ModuleTest extends LinearOpMode
{
    Robot r;
    GroundIntake g;
    Turret t;
    Claw c;
    ExtensionAngle ang;

    @Override
    public void runOpMode() throws InterruptedException
    {
        r=new Robot(this);
        g=r.groundIntake;
        t=r.turret;
        c=r.claw;
        ang=r.angle;

        waitForStart();

        g.setState(GroundIntake.State.INTAKING);
        //ang.setState(ExtensionAngle.State.VECTORING, 1500);
        //t.setState(Turret.State.LEFT, 3000);
        //c.setState(Claw.State.CLOSE, ()-> r.turret.getState()==Turret.State.STOPPED);

        while(opModeIsActive())
        {

        }
    }
}
