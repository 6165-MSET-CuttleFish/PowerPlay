package org.firstinspires.ftc.teamcode.modules.turret;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
@Config
public class TurretTestTele extends LinearOpMode {
    TurretOld turret;
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
        turret=new TurretOld(hardwareMap);
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
            }else if(toggleAutoAlign &&!v4bSlideSpoolSide){
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
            }
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
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);

        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam2.setPipeline(detector= new Detector());
                webcam2.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(detector2=new Detector());
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addLine("waiting for start");
        telemetry.update();
    }
}
