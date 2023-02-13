/*package org.firstinspires.ftc.teamcode.pipelines;

import android.graphics.Bitmap;
import android.os.Build;
*/
/*import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.pytorch.IValue;
import org.pytorch.Module;
import org.pytorch.Tensor;
import org.pytorch.torchvision.TensorImageUtils;

public class ObjectDetectionPipeline extends OpenCvPipeline
{
    Mat currentMat;
    Module module;

    public ObjectDetectionPipeline(Module module)
    {
        this.module=module;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        currentMat.release();
        currentMat=input;
        return currentMat;
    }

    private Bitmap videoInBitmap()
    {
        Bitmap bitmap=null;
        try
        {
            bitmap=Bitmap.createBitmap(currentMat.cols(), currentMat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(currentMat, bitmap);
        }
        catch(CvException e)
        {
        }
        return bitmap;
    }

    public float[] performInference()
    {
        Bitmap bitmap=videoInBitmap();
        Tensor inputTensor= TensorImageUtils.bitmapToFloat32Tensor(bitmap,
                TensorImageUtils.TORCHVISION_NORM_MEAN_RGB,
                TensorImageUtils.TORCHVISION_NORM_STD_RGB);
        IValue[] outputTuple = module.forward(IValue.from(inputTensor)).toTuple();
        Tensor outputTensor = outputTuple[0].toTensor();
        float[] outputs=outputTensor.getDataAsFloatArray();
        return outputs;
    }

    public double getTicks()
    {
        float[] inferences=performInference();
        //determine what is the closest pole

        //use information to get exact amount of ticks

        return 0;
    }
}
*/