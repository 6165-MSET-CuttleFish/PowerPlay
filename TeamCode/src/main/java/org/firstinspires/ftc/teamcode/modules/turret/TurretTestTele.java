package org.firstinspires.ftc.teamcode.modules.turret;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
@Config
public class TurretTestTele extends LinearOpMode {
    Turret turret;
    final double QUICK_POWER=1.0;
    final double SLOW_POWER = 0.125;
    Detector d = new Detector();
    OpenCvCamera w;
    boolean toggleAutoAlign;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime autoAlignTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        turret=new Turret(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        w = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 2"), viewportContainerIds[1]);
        w.setPipeline(d);
        w.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                w.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });
        telemetry.addLine("waiting for start");
        telemetry.update();
        toggleAutoAlign=false;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                toggleAutoAlign=false;
            }else if(gamepad1.left_bumper){
                toggleAutoAlign=true;
            }
            if(toggleAutoAlign&&d.getLocation()!= Detector.Location.MIDDLE) {
                autoAlignTimer.reset();
                telemetry.addData("Turn Turret by ticks: ", d.getShift());
                turret.setState(Turret.State.AUTOALIGN);
            }else if(d.getLocation()!= Detector.Location.MIDDLE){
                turret.setState(Turret.State.IDLE);
            }
            if(d.getLocation()== Detector.Location.MIDDLE) {
                toggleAutoAlign=false;
                telemetry.addData("Distance", d.getDistance());
                turret.setState(Turret.State.IDLE);
                telemetry.addData("time: ", autoAlignTimer.milliseconds());
            }
            telemetry.addData("Location", d.getLocation());
            telemetry.addData("AutoAlign: ", (toggleAutoAlign)?"Enabled":"Disabled");
            telemetry.update();
            turret.update();

        }

    }
}
