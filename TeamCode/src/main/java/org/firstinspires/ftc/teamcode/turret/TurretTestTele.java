package org.firstinspires.ftc.teamcode.turret;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class TurretTestTele extends LinearOpMode {
    DcMotor turret;
    double position;
    boolean toggleAutoAlign;
    public OpenCvWebcam webcam;
    private Detector detector;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        turret= hardwareMap.get(DcMotor.class, "hturret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toggleAutoAlign=false;
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        camInit();
        waitForStart();

        while (opModeIsActive()){
            position=turret.getCurrentPosition();
            if(gamepad1.right_bumper){
                toggleAutoAlign=false;
            }else if(gamepad1.left_bumper){
                toggleAutoAlign=true;
            }
            if(toggleAutoAlign==false){
                if(gamepad1.right_trigger!=1&&gamepad1.left_trigger==1&&turret.getCurrentPosition()<390.0){
                    turret.setPower(0.25);
                }else if(gamepad1.right_trigger==1&&gamepad1.left_trigger!=1&&turret.getCurrentPosition()>-390.0){
                    turret.setPower(-0.25);
                }else{
                    turret.setPower(0);
                }
            }else{
                if(detector.getLocation()== Detector.Location.MIDDLE) {
                    turret.setPower(0);
                    toggleAutoAlign=false;
                }else if(detector.getLocation()== Detector.Location.RIGHT){
                    turret.setPower(0.25);
                }else if(detector.getLocation()== Detector.Location.LEFT){
                    turret.setPower(-0.25);
                }else{
                    turret.setPower(0);
                }
            }
            telemetry.addData("Location", detector.getLocation());
            telemetry.addData("AutoAlign: ", (toggleAutoAlign)?"Enabled":"Disabled");
            telemetry.addData("Current Position", position);
            telemetry.update();

        }

    }
    public void camInit() {
        final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
        final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "rnjlow"), cameraMonitorViewId);
        webcam.setPipeline(detector = new Detector());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                System.out.println("START");
            }
            public void onError(int errorCode) {
            }
        });
        dashboard.startCameraStream(webcam, 30);
        telemetry.addLine("waiting for start");
        telemetry.update();
    }
}
