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
    public static Rect POS_1_HIGH = new Rect(0, 10, 40, 30);
    public static Rect POS_2_HIGH = new Rect(40, 10, 40, 30);
    public static Rect POS_3_HIGH = new Rect(80, 10, 40, 30);
    public static Rect POS_4_HIGH = new Rect(120, 10, 40, 30);
    public static Rect POS_5_HIGH = new Rect(160, 10, 40, 30);
    public static Rect POS_6_HIGH = new Rect(200, 10, 40, 30);
    public static Rect POS_7_HIGH = new Rect(240, 10, 40, 30);
    public static Rect POS_8_HIGH = new Rect(280, 10, 40, 30);

    public static Rect POS_1_LOW = new Rect(0, 170, 40, 30);
    public static Rect POS_2_LOW = new Rect(40, 170, 40, 30);
    public static Rect POS_3_LOW = new Rect(80, 170, 40, 30);
    public static Rect POS_4_LOW = new Rect(120, 170, 40, 30);
    public static Rect POS_5_LOW = new Rect(160, 170, 40, 30);
    public static Rect POS_6_LOW = new Rect(200, 170, 40, 30);
    public static Rect POS_7_LOW = new Rect(240, 170, 40, 30);
    public static Rect POS_8_LOW = new Rect(280, 170, 40, 30);

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

    @Override
    public void init(Mat input)
    {
        recording="Recording";
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV;
        Scalar highHSV;
        lowHSV = new Scalar(HLow, SLow, VLow);
        highHSV = new Scalar(HHigh, SHigh, VHigh);
        Core.inRange(mat, lowHSV, highHSV, mat);

        //rectangle(mat, POS_1_BLUE, new Scalar(255, 255, 255));
        /*
        switch(state)
        {
            case POLE:*/
                rectangle(mat, POS_2_HIGH, new Scalar(255, 255, 255));
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
                rectangle(input, POS_2_HIGH, new Scalar(255, 255, 255));
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
                /*
            case CONESTACK:
                rectangle(mat, POS_2_LOW, new Scalar(255, 255, 255));
                rectangle(mat, POS_3_LOW, new Scalar(255, 255, 255));
                rectangle(mat, POS_4_LOW, new Scalar(255, 255, 255));
                rectangle(mat, POS_5_LOW, new Scalar(255, 255, 255));
                rectangle(mat, POS_6_LOW, new Scalar(255, 255, 255));
                rectangle(mat, POS_7_LOW, new Scalar(255, 255, 255));
                //rectangle(mat, POS_8_BLUE, new Scalar(255, 255, 255));
                //rectangle(input,POS_1_BLUE, new Scalar(255, 255, 255));
                rectangle(input, POS_3_LOW, new Scalar(255, 255, 255));
                rectangle(input, POS_4_LOW, new Scalar(255, 255, 255));
                rectangle(input, POS_5_LOW, new Scalar(255, 255, 255));
                rectangle(input, POS_6_LOW, new Scalar(255, 255, 255));
                rectangle(input, POS_7_LOW, new Scalar(255, 255, 255));
                rectangle(input, POS_2_LOW, new Scalar(255, 255, 255));
                //rectangle(input,POS_8_BLUE, new Scalar(255, 255, 255));

                //loc[0] = Core.sumElems(mat.submat(POS_1_BLUE)).val[0]/(POS_1_BLUE.height*POS_1_BLUE.width*255);
                loc[1] = Core.sumElems(mat.submat(POS_2_LOW)).val[0]/(POS_2_LOW.height* POS_2_LOW.width*255);
                loc[2] = Core.sumElems(mat.submat(POS_3_LOW)).val[0]/(POS_3_LOW.height* POS_3_LOW.width*255);
                loc[3] = Core.sumElems(mat.submat(POS_4_LOW)).val[0]/(POS_4_LOW.height* POS_4_LOW.width*255);
                loc[4] = Core.sumElems(mat.submat(POS_5_LOW)).val[0]/(POS_5_LOW.height* POS_5_LOW.width*255);
                loc[5] = Core.sumElems(mat.submat(POS_6_LOW)).val[0]/(POS_6_LOW.height* POS_6_LOW.width*255);
                loc[6] = Core.sumElems(mat.submat(POS_7_LOW)).val[0]/(POS_7_LOW.height* POS_7_LOW.width*255);
                //loc[7] = Core.sumElems(mat.submat(POS_8_BLUE)).val[0]/(POS_7_BLUE.height*POS_8_BLUE.width*255);
                boxsize =Math.round(((/*loc[0]+loc[1]+loc[3]+loc[4]+loc[5]+loc[6]+loc[2])*50));
                record=Math.abs(loc[3]-loc[4]);
                if(getShift()<restrict) {
                    location= Location.MIDDLE;
                }else if((/*loc[0]+loc[1]+loc[2]+loc[3])<(loc[5]+loc[6]+loc[4]/*loc[7])){
                    location= Location.RIGHT;
                }else{
                    location= Location.LEFT;
                }*/
        //}
        return returnBlack ? mat : input;
    }

    /*public void setState(State s)
    {
        state=s;
    }*/

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
