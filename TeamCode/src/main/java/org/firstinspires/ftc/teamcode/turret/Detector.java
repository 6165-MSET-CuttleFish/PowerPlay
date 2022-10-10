package org.firstinspires.ftc.teamcode.turret;
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
        RIGHT,
    }
    private Location location = Location.MIDDLE;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    // find and set the regions of interest

    public static Rect POS_1_BLUE = new Rect(100, 5, 57, 230);
    public static  Rect POS_2_BLUE = new Rect(163, 5, 57, 230);

    //Find numbers for actual place

    public static int blueHLow = 0;
    public static int blueSLow = 0;
    public static int blueVLow = 100 ;

    public static int blueHHigh = 255;
    public static int blueSHigh = 255;
    public static int blueVHigh = 160;

    public static boolean returnBlack = true;
    private double boxsize =0;
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
        rectangle(input,POS_1_BLUE, new Scalar(255, 255, 255));
        rectangle(input,POS_2_BLUE, new Scalar(255, 255, 255));

        double leftValue = Core.sumElems(mat.submat(POS_1_BLUE)).val[0]/(POS_1_BLUE.height*POS_1_BLUE.width*255);
        double rightValue = Core.sumElems(mat.submat(POS_2_BLUE)).val[0]/(POS_2_BLUE.height*POS_2_BLUE.width*255);
        if(leftValue-rightValue>0.05){
            location=Location.LEFT;
        }else if(leftValue-rightValue<-0.05){
            location=Location.RIGHT;
        }else{
            location=Location.MIDDLE;
        }
        boxsize =Math.round(((leftValue+rightValue)*50));
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
}
