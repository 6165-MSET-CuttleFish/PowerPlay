package org.firstinspires.ftc.teamcode.modules.turret;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
@Config
public class TurretTestTele extends LinearOpMode {
    Turret turret;
    final double QUICK_POWER=1.0;
    final double SLOW_POWER = 0.125;
    boolean toggleAutoAlign;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime autoAlignTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        turret=new Turret(hardwareMap,false);
        toggleAutoAlign=false;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                toggleAutoAlign=false;
            }else if(gamepad1.left_bumper){
                toggleAutoAlign=true;
            }
            if(toggleAutoAlign&&turret.detector.getLocation()!= Detector.Location.MIDDLE) {
                autoAlignTimer.reset();
                telemetry.addData("Turn Turret by ticks: ", turret.detector.getShift());
                turret.setState(Turret.State.AUTOALIGN);
            }else if(turret.detector.getLocation()!= Detector.Location.MIDDLE){
                turret.setState(Turret.State.IDLE);
            }
            if(turret.detector.getLocation()== Detector.Location.MIDDLE) {
                toggleAutoAlign=false;
                telemetry.addData("Distance", turret.detector.getDistance());
                turret.setState(Turret.State.IDLE);
                telemetry.addData("time: ", autoAlignTimer.milliseconds());
            }
            telemetry.addData("Location", turret.detector.getLocation());
            telemetry.addData("AutoAlign: ", (toggleAutoAlign)?"Enabled":"Disabled");
            telemetry.update();
            turret.update();

        }

    }
}
