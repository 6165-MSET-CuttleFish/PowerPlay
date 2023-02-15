package org.firstinspires.ftc.teamcode.modules.turret;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "A Vision Test")
public class DetectionTest extends OpMode {
    Turret turret;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        turret=new Turret(hardwareMap,false);
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("Location", turret.detector.getLocation());
        telemetry.addData("Toggle", gamepad1.left_bumper);
        telemetry.addData("Turret: ", turret.detector.record);
        if(gamepad1.left_bumper&&turret.detector.getLocation()!= Detector.Location.MIDDLE) {
            telemetry.addData("Turn Turret by ticks: ", turret.detector.getShift());
            turret.setState(Turret.State.AUTOALIGN);
        }else if(turret.detector.getLocation()!= Detector.Location.MIDDLE){
            turret.setState(Turret.State.IDLE);
        }
        if(turret.detector.getLocation()== Detector.Location.MIDDLE) {
            telemetry.addData("Distance", turret.detector.getDistance());
            turret.setState(Turret.State.IDLE);
        }
        turret.update();
        telemetry.update();
    }
}
