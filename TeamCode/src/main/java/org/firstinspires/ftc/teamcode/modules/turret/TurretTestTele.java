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
    Turret turret;
    final double QUICK_POWER=1.0;
    final double SLOW_POWER = 0.125;
    boolean toggleAutoAlign;
    public OpenCvWebcam webcam;
    private Detector detector;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        turret=new Turret(hardwareMap,false);
        turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toggleAutoAlign=false;
        turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        camInit();
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                toggleAutoAlign=false;
            }else if(gamepad1.left_bumper){
                toggleAutoAlign=true;
            }
            if(toggleAutoAlign==false) {
                turret.setState(Turret.State.MANUAL);
                turret.turretMotor.setPower(gamepad2.right_stick_y * 0.5);
            }else{
                turret.turretMotor.setTargetPosition(turret.encoder.getCurrentPosition()+detector.getShift());
                turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                toggleAutoAlign=false;
            }
            telemetry.addData("Location", detector.getLocation());
            telemetry.addData("AutoAlign: ", (toggleAutoAlign)?"Enabled":"Disabled");
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

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(detector=new Detector());
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
