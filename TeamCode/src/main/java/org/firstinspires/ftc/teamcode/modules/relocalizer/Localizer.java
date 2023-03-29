package org.firstinspires.ftc.teamcode.modules.relocalizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Side;

public class Localizer
{
    public MB1242 distfl /*, distfr*/;
    public MB1643 left, right;
    

    public double frontDistL;
    //public double frontDistR;
    public double leftDist;
    public double rightDist;

    public double frontDistLRaw, /*frontDistRRaw*/ leftDistRaw, rightDistRaw;

    filter movingMedianFrontL;
    //filter movingMedianFrontR;
    filter movingMedianLeft;
    filter movingMedianRight;

    boolean enabled=true;

    public enum LocalizationMode
    {
        TO_INTAKE,
        INTAKING,
        BACKGROUND,
        OFF
    }

    LocalizationMode mode=LocalizationMode.BACKGROUND;

    Robot r;



    public Localizer(Robot r)
    {
        this.r=r;
        HardwareMap hardwareMap=r.hardwareMap;
        distfl = hardwareMap.get(MB1242.class, "frontLeftDistance");
        //distfr = hardwareMap.get(MB1242.class, "frontRightDistance");
        left = new MB1643(hardwareMap, "left");
        right = new MB1643(hardwareMap, "right");

        movingMedianFrontL=new filter(10);
        //movingMedianFrontR=new filter(10);
        movingMedianLeft=new filter(10);
        movingMedianRight=new filter(10);
    }

    public void update()
    {
        if(mode!=LocalizationMode.OFF)
        {
            frontDistL=movingMedianFrontL.update(distfl.getDistance(DistanceUnit.INCH));
            frontDistLRaw=distfl.getDistance(DistanceUnit.INCH);

            //frontDistR=movingMedianFrontR.update(distfr.getDistance(DistanceUnit.INCH));
            //frontDistRRaw=distfr.getDistance(DistanceUnit.INCH);

            leftDist=movingMedianLeft.update(left.getDistance(DistanceUnit.INCH));
            leftDistRaw=left.getDistance(DistanceUnit.INCH);

            rightDist=movingMedianRight.update(right.getDistance(DistanceUnit.INCH));
            rightDistRaw=right.getDistance(DistanceUnit.INCH);
        }
    }

    public void setMode(LocalizationMode mode)
    {
        this.mode=mode;
    }

    public void relocalize()
    {
        switch(mode)
        {
            case TO_INTAKE:
                if(frontDistLRaw<10)
                {
                    r.setPoseEstimate(new Pose2d(-56.8, r.getPoseEstimate().getY(), r.getPoseEstimate().getHeading()));
                }
            case INTAKING:
                /*if(Context.side==Side.LEFT)
                {
                    double y=(leftDist-4)+15;
                    r.setPoseEstimate(new Pose2d(r.getPoseEstimate().getX(), y, r.getPoseEstimate().getHeading()));
                }
                else if(Context.side==Side.RIGHT)
                {
                    double y=0;
                    r.setPoseEstimate(new Pose2d(r.getPoseEstimate().getX(), y, r.getPoseEstimate().getHeading()));
                }*/
        }
    }
}
