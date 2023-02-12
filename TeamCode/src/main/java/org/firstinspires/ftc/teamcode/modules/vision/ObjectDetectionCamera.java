package org.firstinspires.ftc.teamcode.modules.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.ObjectDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.colorDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.pytorch.Module;

public class ObjectDetectionCamera
{
    OpenCvWebcam camera;
    ObjectDetectionPipeline pipeline;
    Module module;
    HardwareMap hardwareMap;


    public ObjectDetectionCamera(HardwareMap map, String modelFilePath)
    {
        hardwareMap=map;
        accessModelFile(modelFilePath);
        pipeline=new ObjectDetectionPipeline(module);
    }

    public void initCamera()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline=new ObjectDetectionPipeline(module);
        camera.setPipeline(pipeline);
        camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });
    }

    public void closeCamera()
    {
        camera.stopStreaming();
    }

    private void accessModelFile(String filePath)
    {

    }
    public double tickDistance()
    {
        return pipeline.getTicks();
    }
}
