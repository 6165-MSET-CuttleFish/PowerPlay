package org.firstinspires.ftc.teamcode.turret;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ModuleTest;

@TeleOp
public class TurretTest extends ModuleTest {
    Turret turret;
    double targetPosition = 0;

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap) {
            @Override
            public void setPower(float v) {

            }

            @Override
            public void init() {
                setModules(turret);
            }

            @Override
            public void setDebugMode(boolean b) {

            }
        };
    }

    @Override
    public void update() {
        turret.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
