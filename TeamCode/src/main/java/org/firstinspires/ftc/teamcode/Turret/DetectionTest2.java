package org.firstinspires.ftc.teamcode.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "Vision Test Double")
public class DetectionTest2 extends OpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    public OpenCvWebcam webcam2;
    private Detector detector2;
    public OpenCvWebcam webcam;
    private Detector detector;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        int cameraMonitorViewId2 = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId2",
                        "id2",
                        hardwareMap.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector = new Detector());
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam2 = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId2);
        webcam2.setPipeline(detector2 = new Detector());
        webcam2.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                System.out.println("START");
            }
            public void onError(int errorCode) {
            }
        });
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                System.out.println("START");
            }
            public void onError(int errorCode) {
            }
        });
        dashboard.startCameraStream(webcam, 30);
        dashboard.startCameraStream(webcam2, 30);
        telemetry.addLine("waiting for start");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Location", detector.getLocation());
        if(detector.getLocation()== Detector.Location.MIDDLE) {
            telemetry.addData("Distance", detector.getDistance());
        }
        if(detector2.getLocation()== Detector.Location.MIDDLE) {
            telemetry.addData("Distance 2", detector2.getDistance());
        }
        telemetry.update();
    }
}
