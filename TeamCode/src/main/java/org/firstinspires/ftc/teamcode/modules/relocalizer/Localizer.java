package org.firstinspires.ftc.teamcode.modules.relocalizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Side;

public class Localizer
{

    public MB1242Ex distfl /*, distfr*/;

    public MB1643 left, right;


    public double frontDistL;
    //public double frontDistR;
    public double leftDist;
    public double rightDist;
    public double localizedCount=0;

    public double frontDistLRaw, /*frontDistRRaw*/ leftDistRaw, rightDistRaw;

    filter movingMedianFrontL;
    //filter movingMedianFrontR;
    filter movingMedianLeft;
    filter movingMedianRight;

    public double savedDist;

    boolean enabled=true;

    public enum LocalizationMode
    {
        TO_INTAKE,
        INTAKING,
        BACKGROUND,
        OFF,
        COMPLETED
    }

    LocalizationMode mode=LocalizationMode.BACKGROUND;

    Robot r;

    public boolean cancelTraj=false;


    public Localizer(Robot r)
    {
        this.r=r;
        HardwareMap hardwareMap=r.hardwareMap;
        //hardwareMap.getAll(I2cDeviceSynch.class);

        distfl = hardwareMap.get(MB1242Ex.class, "frontLeftDistance");
        //distfl.doInitialize();
        //distfl.resetDeviceConfigurationForOpMode();
        //distfl.setRunDelayMs(5);

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
            frontDistLRaw=distfl.getDistanceAsync(DistanceUnit.INCH);
            frontDistL=movingMedianFrontL.update(frontDistLRaw);

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
        if(frontDistLRaw<18&&frontDistLRaw>7)
        {
            if(mode==LocalizationMode.TO_INTAKE)
            {
                //savedDist=frontDistLRaw;
                //r.setPoseEstimate(new Pose2d(-56.8, 13.75, r.getPoseEstimate().getHeading()));
                r.breakFollowing();
                //r.setPoseEstimate(new Pose2d(-56.8, 13.75, r.getPoseEstimate().getHeading()));

                cancelTraj=true;
                mode=LocalizationMode.BACKGROUND;
                localizedCount++;
            }
            else if(mode==LocalizationMode.COMPLETED)
            {
                cancelTraj=false;
            }
        }
        else
        {
            cancelTraj=false;
        }
    }
}
