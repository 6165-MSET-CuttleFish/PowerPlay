package org.firstinspires.ftc.teamcode.detection.pipelines

import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.detection.visionProcessing.SignalSleeveProcessor
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

//new more efficient one that might have problems
class colorDetectionPipeline: OpenCvPipeline()
{
    private val detector: SignalSleeveProcessor = SignalSleeveProcessor();
    private var currentMat:Mat?=Mat();

    override fun processFrame(input: Mat?): Mat?
    {
        currentMat=input;
        return input;
    }

    fun getState(): Int
    {
        val state=runBlocking(Dispatchers.Default)
        {
            detector.getState(currentMat);
        }
        return state;
    }
}