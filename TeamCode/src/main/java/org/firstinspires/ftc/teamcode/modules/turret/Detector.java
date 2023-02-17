package org.firstinspires.ftc.teamcode.modules.turret;
import static org.opencv.imgproc.Imgproc.rectangle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class Detector extends OpenCvPipeline {
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location location = Location.MIDDLE;
    public static double factor=-75;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    // find and set the regions of interest

    public static Rect POS_1_BLUE = new Rect(0, 200, 40, 40);
    public static Rect POS_2_BLUE = new Rect(40, 200, 40, 40);
    public static Rect POS_3_BLUE = new Rect(80, 200, 40, 40);
    public static Rect POS_4_BLUE = new Rect(120, 200, 40, 40);
    public static Rect POS_5_BLUE = new Rect(160, 200, 40, 40);
    public static Rect POS_6_BLUE = new Rect(200, 200, 40, 40);
    public static Rect POS_7_BLUE = new Rect(240, 200, 40, 40);
    public static Rect POS_8_BLUE = new Rect(280, 200, 40, 40);

    //Find numbers for actual place

    public static int blueHLow = 20;
    public static int blueSLow = 100;
    public static int blueVLow = 0;

    public static int blueHHigh = 255;
    public static int blueSHigh = 255;
    public static int blueVHigh = 255;

    public static boolean returnBlack = true;
    private double boxsize =0;
    public double record;
    public static double restrict=0.6;
    private double[] loc=new double[8];
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV;
        Scalar highHSV;
        lowHSV = new Scalar(blueHLow, blueSLow, blueVLow);
        highHSV = new Scalar(blueHHigh, blueSHigh, blueVHigh);
        Core.inRange(mat, lowHSV, highHSV, mat);

        rectangle(mat, POS_1_BLUE, new Scalar(255, 255, 255));
        rectangle(mat, POS_2_BLUE, new Scalar(255, 255, 255));
        rectangle(mat, POS_3_BLUE, new Scalar(255, 255, 255));
        rectangle(mat, POS_4_BLUE, new Scalar(255, 255, 255));
        rectangle(mat, POS_5_BLUE, new Scalar(255, 255, 255));
        rectangle(mat, POS_6_BLUE, new Scalar(255, 255, 255));
        rectangle(mat, POS_7_BLUE, new Scalar(255, 255, 255));
        rectangle(mat, POS_8_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_1_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_3_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_4_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_5_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_6_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_7_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_2_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_8_BLUE, new Scalar(255, 255, 255));

        loc[0] = Core.sumElems(mat.submat(POS_1_BLUE)).val[0]/(POS_1_BLUE.height*POS_1_BLUE.width*255);
        loc[1] = Core.sumElems(mat.submat(POS_2_BLUE)).val[0]/(POS_2_BLUE.height*POS_2_BLUE.width*255);
        loc[2] = Core.sumElems(mat.submat(POS_3_BLUE)).val[0]/(POS_3_BLUE.height*POS_3_BLUE.width*255);
        loc[3] = Core.sumElems(mat.submat(POS_4_BLUE)).val[0]/(POS_4_BLUE.height*POS_4_BLUE.width*255);
        loc[4] = Core.sumElems(mat.submat(POS_5_BLUE)).val[0]/(POS_5_BLUE.height*POS_5_BLUE.width*255);
        loc[5] = Core.sumElems(mat.submat(POS_6_BLUE)).val[0]/(POS_6_BLUE.height*POS_6_BLUE.width*255);
        loc[6] = Core.sumElems(mat.submat(POS_7_BLUE)).val[0]/(POS_7_BLUE.height*POS_7_BLUE.width*255);
        loc[7] = Core.sumElems(mat.submat(POS_8_BLUE)).val[0]/(POS_7_BLUE.height*POS_8_BLUE.width*255);
        boxsize =Math.round(((loc[0]+loc[1]+loc[3]+loc[4]+loc[5]+loc[6]+loc[2])*50));
        record=Math.abs(loc[3]-loc[4]);
        if(Math.abs(loc[3]-loc[4])<restrict&&loc[3]>0.26&&loc[4]>0.26) {
            location= Location.MIDDLE;
        }else if((loc[0]+loc[1]+loc[2]+loc[3])<(loc[5]+loc[6]+loc[4]+loc[7])){
            location= Location.RIGHT;
        }else{
            location= Location.LEFT;
        }
        return returnBlack ? mat : input;

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
        return (int)(factor*(loc[0]*4+loc[1]*3+loc[2]*2+loc[3]*1+loc[4]*-1+loc[5]*-2+loc[6]*-3));
    }
}
