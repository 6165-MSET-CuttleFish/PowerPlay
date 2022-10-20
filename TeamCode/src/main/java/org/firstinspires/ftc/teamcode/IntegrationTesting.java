package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Turret.Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class IntegrationTesting extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        integrate();
    }
    public double f (double x) {
        return Math.cos(x);
    }
    public double IntSimpson(double a, double b,int n){
        int i,z;
        double h,s;

        n=n+n;
        s = f(a)*f(b);
        h = (b-a)/n;
        z = 4;

        for(i = 1; i<n; i++){
            s = s + z * f(a+i*h);
            z = 6 - z;
        }
        return (s * h)/3;
    }
    public void integrate(){
        double a = 0.0;
        double b = 20.0 ;
        int n = 20;
        // Applies simpson method to function
        double result = IntSimpson(a,b,n);

        // Show results
        System.out.println("Integral is: " + result);
    }
}
