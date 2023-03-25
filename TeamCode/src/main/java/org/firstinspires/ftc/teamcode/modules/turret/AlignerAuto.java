package org.firstinspires.ftc.teamcode.modules.turret;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2RGB;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.rectangle;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
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
    public static int factor=100;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int boxHeight=120;
    public static double ratio=1.8;

    // find and set the regions of interest
    public static Rect POS_1_HIGH = new Rect(0, 0, 40, boxHeight);
    public Rect POS_2_HIGH = new Rect(40, 0, 40, boxHeight);
    public Rect POS_3_HIGH = new Rect(80, 0, 40, boxHeight);
    public Rect POS_4_HIGH = new Rect(120, 0, 40, boxHeight);
    public Rect POS_5_HIGH = new Rect(160, 0, 40, boxHeight);
    public Rect POS_6_HIGH = new Rect(200, 0, 40, boxHeight);
    public Rect POS_7_HIGH = new Rect(240, 0, 40, boxHeight);
    //public static Rect POS_8_HIGH = new Rect(280, 0, 40, 30);

    public Rect HigherBoxes=new Rect(0, 10, 320, boxHeight);
    public Rect LowerBoxes=new Rect(0, 170, 320, boxHeight);

    //Find numbers for actual place

    public static int HLow = 5;
    public static int SLow = 80;
    public static int VLow = 50;

    public static int HHigh = 55;
    public static int SHigh = 255;
    public static int VHigh = 255;

    public static boolean returnBlack = true;
    private double boxsize =0;
    public double record;
    public static double restrict=0.6;
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

    Mat submat_2=new Mat();
    Mat submat_3=new Mat();
    Mat submat_4=new Mat();
    Mat submat_5=new Mat();
    Mat submat_6=new Mat();
    Mat submat_7=new Mat();

    Scalar lowHSV;
    Scalar highHSV;

    public double centerX=-1;



    @Override
    public void init(Mat input)
    {
        selectedBoxes=input.submat(HigherBoxes);

        recording="Recording";
        maskTemplate=new Mat(selectedBoxes.rows(), selectedBoxes.cols(), CvType.CV_8U, Scalar.all(0));

        lowHSV = new Scalar(HLow, SLow, VLow);
        highHSV = new Scalar(HHigh, SHigh, VHigh);

        kernel=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(13, 13));
        kernel2=Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));
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

        submat_2.release();
        submat_3.release();
        submat_4.release();
        submat_5.release();
        submat_6.release();
        submat_7.release();

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
                int height=Imgproc.boundingRect(Contours.get(i)).height;
                int width=Imgproc.boundingRect(Contours.get(i)).width;
                //Imgproc.boundingRect(Contours.get(i)).br();
                double area=Imgproc.contourArea(Contours.get(i));

                if(area>contourArea&&width*ratio<height)
                {
                    contourArea=area;
                    contourIndex=i;
                }
            }
            if(Contours.size()>0)
            {
                mat=maskTemplate.clone();
                Imgproc.drawContours(mat, Contours, contourIndex, new Scalar(255, 255, 255), -1);
                centerX=(Imgproc.boundingRect(Contours.get(contourIndex)).x+Imgproc.boundingRect(Contours.get(contourIndex)).br().x)/2;
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
        submat_2=mat.submat(POS_2_HIGH);
        submat_3=mat.submat(POS_3_HIGH);
        submat_4=mat.submat(POS_4_HIGH);
        submat_5=mat.submat(POS_5_HIGH);
        submat_6=mat.submat(POS_6_HIGH);
        submat_7=mat.submat(POS_7_HIGH);
        loc[1] = Core.sumElems(submat_2).val[0]/(POS_2_HIGH.height* POS_2_HIGH.width*255);
        loc[2] = Core.sumElems(submat_3).val[0]/(POS_3_HIGH.height* POS_3_HIGH.width*255);
        loc[3] = Core.sumElems(submat_4).val[0]/(POS_4_HIGH.height* POS_4_HIGH.width*255);
        loc[4] = Core.sumElems(submat_5).val[0]/(POS_5_HIGH.height* POS_5_HIGH.width*255);
        loc[5] = Core.sumElems(submat_6).val[0]/(POS_6_HIGH.height* POS_6_HIGH.width*255);
        loc[6] = Core.sumElems(submat_7).val[0]/(POS_7_HIGH.height* POS_7_HIGH.width*255);
                //loc[7] = Core.sumElems(mat.submat(POS_8_BLUE)).val[0]/(POS_7_BLUE.height*POS_8_BLUE.width*255);
        boxsize =Math.round(((/*loc[0]*/+loc[1]+loc[3]+loc[4]+loc[5]+loc[6]+loc[2])*50));
        record=Math.abs(loc[3]-loc[4]);
        if(getShift()<restrict)
        {
            location= Location.MIDDLE;
        }else if((/*loc[0]*/+loc[1]+loc[2]+loc[3])<(loc[5]+loc[6]+loc[4]/*loc[7]*/)){
            location= Location.RIGHT;
        }else{
            location= Location.LEFT;
        }


        //Imgproc.cvtColor(mat, returnMat, Imgproc.COLOR_HSV2RGB);
        return mat;
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
    public int getShift()
    {
        //Find shift
        return (int)(factor*(loc[1]*3+loc[2]*2+loc[3]*1+loc[4]*-1+loc[5]*-2+loc[6]*-3));
    }
}
