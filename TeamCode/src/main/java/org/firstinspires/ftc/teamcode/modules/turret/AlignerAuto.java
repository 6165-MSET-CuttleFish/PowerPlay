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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class AlignerAuto extends OpenCvPipeline {
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
    public static int factor=75;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    // find and set the regions of interest
    public static Rect POS_1_HIGH = new Rect(0, 0, 40, 30);
    public static Rect POS_2_HIGH = new Rect(40, 0, 40, 30);
    public static Rect POS_3_HIGH = new Rect(80, 0, 40, 30);
    public static Rect POS_4_HIGH = new Rect(120, 0, 40, 30);
    public static Rect POS_5_HIGH = new Rect(160, 0, 40, 30);
    public static Rect POS_6_HIGH = new Rect(200, 0, 40, 30);
    public static Rect POS_7_HIGH = new Rect(240, 0, 40, 30);
    public static Rect POS_8_HIGH = new Rect(280, 0, 40, 30);

    public static Rect HigherBoxes=new Rect(0, 10, 320, 30);
    public static Rect LowerBoxes=new Rect(0, 170, 320, 30);

    //Find numbers for actual place

    public static int HLow = 0;
    public static int SLow = 100;
    public static int VLow = 120;

    public static int HHigh = 255;
    public static int SHigh = 255;
    public static int VHigh = 255;

    public static boolean returnBlack = true;
    private double boxsize =0;
    public double record;
    public static double restrict=0.6;
    public String recording= "Not Recording";
    private double[] loc=new double[8];

    Mat processed=new Mat();
    Mat morphed=new Mat();
    Mat kernel=new Mat();
    Mat morphed2=new Mat();
    Mat kernel2=new Mat();
    List<MatOfPoint> Contours=new ArrayList<MatOfPoint>();
    Mat hierarchy=new Mat();
    Mat mask=new Mat();
    Mat selectedBoxes=new Mat();
    Mat inRange=new Mat();


    @Override
    public void init(Mat input)
    {
        recording="Recording";
    }

    public void release()
    {
        processed.release();
        morphed.release();
        kernel.release();
        morphed2.release();
        kernel2.release();
        hierarchy.release();
        mask.release();
        selectedBoxes.release();
        inRange.release();
    }

    @Override
    public Mat processFrame(Mat input)
    {
        release();

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV;
        Scalar highHSV;
        lowHSV = new Scalar(HLow, SLow, VLow);
        highHSV = new Scalar(HHigh, SHigh, VHigh);

        if(state==State.POLE)
        {
            selectedBoxes=mat.submat(HigherBoxes);
        }
        else
        {
            selectedBoxes=mat.submat(LowerBoxes);
        }

        Imgproc.bilateralFilter(selectedBoxes, processed, 15, 75, 75);
        Core.inRange(processed, lowHSV, highHSV, inRange);

        kernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(inRange, morphed, MORPH_OPEN, kernel);

        kernel2=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9));
        Imgproc.morphologyEx(morphed, morphed2, MORPH_CLOSE, kernel2);

        Imgproc.findContours(morphed2, Contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int contourIndex=0;
        double contourArea=0;

        for(int i=0; i<Contours.size(); i++)
        {
            if(Imgproc.contourArea(Contours.get(i))>contourArea)
            {
                contourArea=Imgproc.contourArea(Contours.get(i));
                contourIndex=i;
            }
        }

        if(Contours.size()>0)
        {
            mask=new Mat(selectedBoxes.rows(), selectedBoxes.cols(), CvType.CV_8U, Scalar.all(0));
            Imgproc.drawContours(mask, Contours, contourIndex, new Scalar(255, 255, 255), -1);
            //Rect rect=Imgproc.boundingRect(Contours.get(contourIndex));
            mat=mask.clone();
            //Core.bitwise_and(laCringe, laCringe, finalMat, Contours.get(contourIndex));
            //Imgproc.rectangle(finalMat, rect, new Scalar (0, 255, 0));
        }

        //rectangle(mat, POS_1_BLUE, new Scalar(255, 255, 255));


        /*rectangle(mat, POS_2_HIGH, new Scalar(255, 255, 255));
        rectangle(mat, POS_3_HIGH, new Scalar(255, 255, 255));
        rectangle(mat, POS_4_HIGH, new Scalar(255, 255, 255));
        rectangle(mat, POS_5_HIGH, new Scalar(255, 255, 255));
        rectangle(mat, POS_6_HIGH, new Scalar(255, 255, 255));
        rectangle(mat, POS_7_HIGH, new Scalar(255, 255, 255));
                //rectangle(mat, POS_8_BLUE, new Scalar(255, 255, 255));
                //rectangle(input,POS_1_BLUE, new Scalar(255, 255, 255));
        rectangle(input, POS_3_HIGH, new Scalar(255, 255, 255));
        rectangle(input, POS_4_HIGH, new Scalar(255, 255, 255));
        rectangle(input, POS_5_HIGH, new Scalar(255, 255, 255));
        rectangle(input, POS_6_HIGH, new Scalar(255, 255, 255));
        rectangle(input, POS_7_HIGH, new Scalar(255, 255, 255));
        rectangle(input, POS_2_HIGH, new Scalar(255, 255, 255));*/
                //rectangle(input,POS_8_BLUE, new Scalar(255, 255, 255));

                //loc[0] = Core.sumElems(mat.submat(POS_1_BLUE)).val[0]/(POS_1_BLUE.height*POS_1_BLUE.width*255);
        loc[1] = Core.sumElems(mat.submat(POS_2_HIGH)).val[0]/(POS_2_HIGH.height* POS_2_HIGH.width*255);
        loc[2] = Core.sumElems(mat.submat(POS_3_HIGH)).val[0]/(POS_3_HIGH.height* POS_3_HIGH.width*255);
        loc[3] = Core.sumElems(mat.submat(POS_4_HIGH)).val[0]/(POS_4_HIGH.height* POS_4_HIGH.width*255);
        loc[4] = Core.sumElems(mat.submat(POS_5_HIGH)).val[0]/(POS_5_HIGH.height* POS_5_HIGH.width*255);
        loc[5] = Core.sumElems(mat.submat(POS_6_HIGH)).val[0]/(POS_6_HIGH.height* POS_6_HIGH.width*255);
        loc[6] = Core.sumElems(mat.submat(POS_7_HIGH)).val[0]/(POS_7_HIGH.height* POS_7_HIGH.width*255);
                //loc[7] = Core.sumElems(mat.submat(POS_8_BLUE)).val[0]/(POS_7_BLUE.height*POS_8_BLUE.width*255);
        boxsize =Math.round(((/*loc[0]*/+loc[1]+loc[3]+loc[4]+loc[5]+loc[6]+loc[2])*50));
        record=Math.abs(loc[3]-loc[4]);
        if(getShift()<restrict) {
            location= Location.MIDDLE;
        }else if((/*loc[0]*/+loc[1]+loc[2]+loc[3])<(loc[5]+loc[6]+loc[4]/*loc[7]*/)){
            location= Location.RIGHT;
        }else{
            location= Location.LEFT;
        }

        return returnBlack ? mat : input;
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
    public int getShift() {
        //Find shift
        return (int)(-1*factor*(loc[0]*4+loc[1]*3+loc[2]*2+loc[3]*1+loc[4]*0+loc[5]*-1+loc[6]*-2));
    }
}
