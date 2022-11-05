package org.firstinspires.ftc.teamcode.Turret;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    public static boolean v4bSlideSpoolSide;
    public OpenCvWebcam webcam;
    private Detector detector;
    public OpenCvWebcam webcam2;
    private Detector detector2;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        turret=new Turret(hardwareMap);
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toggleAutoAlign=false;
        turret.prevPositionReset=0;
        v4bSlideSpoolSide=true;
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        camInit();
        waitForStart();
        while (opModeIsActive()){
            turret.position=turret.turretMotor.getCurrentPosition()-turret.prevPositionReset;
            if(gamepad1.right_bumper){
                toggleAutoAlign=false;
            }else if(gamepad1.left_bumper){
                toggleAutoAlign=true;
            }
            if(turret.magnetic.isPressed()){
                turret.prevPositionReset=turret.position;
                turret.position=0;
            }
            if(toggleAutoAlign==false) {
                if (gamepad1.right_trigger != 1 && gamepad1.left_trigger == 1 && turret.turretMotor.getCurrentPosition() < 390) {
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(QUICK_POWER);
                } else if (gamepad1.right_trigger == 1 && gamepad1.left_trigger != 1 && turret.turretMotor.getCurrentPosition() > -390) {
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(-QUICK_POWER);
                } else {
                    turret.turretMotor.setTargetPosition(turret.position);
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }else if(toggleAutoAlign &&v4bSlideSpoolSide){
                if(detector.getLocation()== Detector.Location.LEFT){
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(SLOW_POWER);
                }else if(detector.getLocation()== Detector.Location.RIGHT){
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.turretMotor.setPower(-SLOW_POWER);
                }else{
                    turret.turretMotor.setTargetPosition(turret.position);
                    turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }/*else if(toggleAutoAlign &&!v4bSlideSpoolSide){
                if(detector2.getLocation()== Detector.Location.LEFT){
                    turret.turretMotor.setPower(SLOW_POWER);
                }else if(detector2.getLocation()== Detector.Location.RIGHT){
                    turret.turretMotor.setPower(-SLOW_POWER);
                }else{
                    if (turret.turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turret.turretMotor.setTargetPosition(turret.position);
                    }
                    if (turret.turretMotor.getCurrentPosition() < turret.position) {
                        turret.turretMotor.setPower(SLOW_POWER);
                    } else if (turret.turretMotor.getCurrentPosition() > turret.position) {
                        turret.turretMotor.setPower(-SLOW_POWER);
                    } else {
                        turret.turretMotor.setPower(0);
                    }
                }
            }*/
            telemetry.addData("Location", detector.getLocation());
            telemetry.addData("AutoAlign: ", (toggleAutoAlign)?"Enabled":"Disabled");
            telemetry.addData("Current Position", turret.position);
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
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
