package org.firstinspires.ftc.teamcode.modules.turret;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.rectangle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class Autoalign extends OpenCvPipeline {
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public enum State
    {
        POLE,
        CONESTACK
    }

    private Location location = Location.MIDDLE;
    private State state=State.POLE;
    public static int factor=70;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int boxHeight=120;
    public static double ratio=1.5;


    public Rect HigherBoxes=new Rect(0, 10, 320, boxHeight);
    public Rect LowerBoxes=new Rect(0, 170, 320, boxHeight);

    //Find numbers for actual place

    public static int HLow = 5;
    public static int SLow = 30;
    public static int VLow = 30;

    public static int HHigh = 55;
    public static int SHigh = 255;
    public static int VHigh = 255;

    public static boolean returnBlack = true;
    private double boxsize =0;
    public String recording= "Not Recording";
    public String error="None";
    private double[] loc=new double[8];

    Mat processed=new Mat();
    Mat morphed=new Mat();
    Mat kernel=new Mat();
    Mat morphed2=new Mat();
    Mat kernel2=new Mat();
    List<MatOfPoint> Contours=new ArrayList<MatOfPoint>();
    Mat hierarchy=new Mat();
    //Mat mask=new Mat();
    Mat selectedBoxes=new Mat();
    Mat inRange=new Mat();
    Mat maskTemplate=new Mat();
    Mat mat=new Mat();
    Mat returnMat=new Mat();
    Mat HSV=new Mat();

    Scalar lowHSV;
    Scalar highHSV;

    public double centerX=-1;
    public double largestArea=0;
    public double threshold=1500;

    public double power=0;

    public boolean aligning=false;
    public boolean alignstate=false;
    Moments m;

    public double highpower=0.35;
    public double lowpower=0.2;
    public static boolean showPole=true;

    public static double multiplier=-1.5;


    @Override
    public void init(Mat input)
    {
        selectedBoxes=input.submat(HigherBoxes);

        recording="Recording";
        maskTemplate=new Mat(selectedBoxes.rows(), selectedBoxes.cols(), CvType.CV_8U, Scalar.all(0));

        lowHSV = new Scalar(HLow, SLow, VLow);
        highHSV = new Scalar(HHigh, SHigh, VHigh);

        kernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(17, 17));
        kernel2=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(13, 13));
    }

    public void release()
    {
        processed.release();
        morphed.release();
        //kernel.release();
        morphed2.release();
        //kernel2.release();
        hierarchy.release();
        //mask.release();
        selectedBoxes.release();
        inRange.release();
        mat.release();
        returnMat.release();
        HSV.release();

        /*submat_2.release();
        submat_3.release();
        submat_4.release();
        submat_5.release();
        submat_6.release();
        submat_7.release();*/

        if(Contours.size()>0)
        {
            Contours.clear();
        }
    }

    @Override
    public Mat processFrame(Mat input)
    {
        release();

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        if(state==State.POLE)
        {
            selectedBoxes=HSV.submat(HigherBoxes);
        }
        else
        {
            selectedBoxes=HSV.submat(LowerBoxes);
        }


        //Imgproc.bilateralFilter(selectedBoxes, processed, 15, 75, 75);
        Core.inRange(selectedBoxes, lowHSV, highHSV, inRange);

        Imgproc.morphologyEx(inRange, morphed, MORPH_OPEN, kernel);

        Imgproc.morphologyEx(morphed, morphed2, MORPH_CLOSE, kernel2);

        Imgproc.findContours(morphed2, Contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int contourIndex=0;
        double contourArea=0;

        for(int i=0; i<Contours.size(); i++)
        {
            Rect boundingRect=Imgproc.boundingRect(Contours.get(i));
            int height=boundingRect.height;
            int width=boundingRect.width;
            //Imgproc.boundingRect(Contours.get(i)).br();
            double area=Imgproc.contourArea(Contours.get(i));

            if(area>contourArea&&width*ratio<height&&area>threshold)
            {
                contourArea=area;
                contourIndex=i;
            }
        }
        if(Contours.size()>0)
        {
            mat=maskTemplate.clone();
            Imgproc.drawContours(mat, Contours, contourIndex, new Scalar(255, 255, 255), -1);
            m=Imgproc.moments(Contours.get(contourIndex));
            if(m.m00!=0)
            {
                centerX=m.m10/m.m00;
            }
            else
            {
                centerX=-1;
            }
            largestArea=contourArea;
            //Rect rect=Imgproc.boundingRect(Contours.get(contourIndex));
            //Core.bitwise_and(laCringe, laCringe, finalMat, Contours.get(contourIndex));
            //Imgproc.rectangle(finalMat, rect, new Scalar (0, 255, 0));
        }
        else
        {
            mat=morphed2;
            centerX=-1;
        }
        error="None";

        //String logFilePath = String.format("%s/FIRST/data/img"+imgCount+".png", Environment.getExternalStorageDirectory().getAbsolutePath());
        //Imgcodecs.imwrite(logFilePath, mat);

        if(alignstate)
        {
            updatePower();
        }
        //Imgproc.cvtColor(mat, returnMat, Imgproc.COLOR_HSV2RGB);
        if(showPole)
            return mat;
        else
            return morphed2;
    }

    public void setState(State s)
    {
        state=s;
    }

    public Location getLocation() {
        return location;
    }

    public double getDistance() {
        //Find equation for actual place
        double distance=Math.round(-2*boxsize+62);
        return distance;
    }

    public double getArea() {
        //Find equation for actual place
        double area=boxsize;
        return area;
    }

    public double getShift()
    {
        if(centerX>-1)
        {
            return (centerX-160)*multiplier;
        }
        else
        {
            return 0;
        }
    }

    //not used either
    private void updatePower()
    {
        if(Math.abs(centerX-160)<5)
        {
            aligning=false;
        }
        else
        {
            aligning=true;
        }

        if(centerX>-1&&aligning)
        {
            if(Math.abs(centerX-160)>30)
            {
                power=-highpower*Math.signum(centerX-160);
            }
            else if(Math.abs(centerX-160)>10)
            {
                power=-lowpower*Math.signum(centerX-160);
            }
            else
            {
                power=0;
            }
        }
        else
        {
            power=0;
        }
    }

    public double getPower()
    {
        return power;
    }
}
