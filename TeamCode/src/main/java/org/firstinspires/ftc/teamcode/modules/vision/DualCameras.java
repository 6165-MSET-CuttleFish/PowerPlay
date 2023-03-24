package org.firstinspires.ftc.teamcode.modules.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.turret.AlignerAuto;
import org.firstinspires.ftc.teamcode.pipelines.colorDetection;
import org.firstinspires.ftc.teamcode.util.Context;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class DualCameras
{
    public OpenCvWebcam autoCamera;
    public OpenCvWebcam turretCamera;
    public AlignerAuto aligner;
    //detector2.setState
    public colorDetection signalSleeve;

    int cameraMonitorViewId;
    int[] viewportContainerIds;

    HardwareMap hardwareMap;
    Robot r;

    public DualCameras(Robot r)
    {
        hardwareMap=r.hardwareMap;
        this.r=r;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        aligner = new AlignerAuto();
        initAutoAlignCamera();

        signalSleeve = new colorDetection();
        initSignalSleeveCamera();
    }

    public void initSignalSleeveCamera()
    {
        if(Context.isAuto)
        {
            autoCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
            autoCamera.setPipeline(signalSleeve);
            autoCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    //telemetry.addData("Camera 1: ", "started");
                    //telemetry.update();
                    autoCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    Context.signalsleeveCameraPastInit=true;
                    //telemetry.addData("Camera 1: ", "streaming");
                    //telemetry.update();
                }

                @Override
                public void onError(int errorCode) {
                    //telemetry.addData("Webcam 1", "failed");
                    //telemetry.update();
                }
            });
        }
    }

    public void initAutoAlignCamera()
    {
        if(Context.autoalignEnabled)
        {
            turretCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
            turretCamera.setPipeline(aligner);
            turretCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    turretCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    r.turret.setAutoalignCamera(aligner);
                    Context.autoalignCameraPastInit=true;
                    //Context.autoalignCameraPastInit=true;
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }
    }

    public void closeCameras()
    {
        autoCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
            }
        });
        turretCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });
    }
}
