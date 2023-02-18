package org.firstinspires.ftc.teamcode.modules.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.util.BPIDFController;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.moduleUtil.HwModule;
import org.firstinspires.ftc.teamcode.util.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.util.PIDCoeff;
import org.firstinspires.ftc.teamcode.util.PIDControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class Turret extends HwModule
{
    public static double p = 0.001, i = 0.0030, d = 0.00006;
    public static double kV = 0, kA = 0, kStatic = 0;
    public BPIDFController pidController;

    public static ElapsedTime time = new ElapsedTime();

    PIDControl controller;
    PIDCoeff coeff;

    public static double offset=8;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    public static int LEFT_POS = 2100, RIGHT_POS = -2100, ZERO_POS = 0, INIT=1020,
            BACK = 4125, RIGHT_DIAGONAL = -3000, LEFT_DIAGONAL = 3000,  RIGHT_SIDE_HIGH = -3100,
            RIGHT_SIDE_HIGH_PRELOAD = -860, RIGHT_SIDE_MID = 3000;



    public static double closePower = 0.3;
    public static double farPower = 0.8;
    double targetPos=0;
    public double posAtZero=0;
    public double prevHall=0;
    public DcMotorEx turretMotor;
    public Encoder encoder;
    public AnalogInput hallEffect;
    private OpenCvWebcam webcam;
    public Detector detector;
    public Turret.State state;
    private boolean isAuto = false;
    public double motorOil=0;

    @Override
    public void setState(ModuleState s)
    {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(s.getClass()==Turret.State.class)
        {
            state=(State)s;
        }
        updateTarget();
        time.reset();
    }

    public enum State implements ModuleState
    {
        IDLE, LEFT, RIGHT, ZERO, MANUAL, AUTOALIGN, INIT, BACK,
        RIGHT_SIDE_HIGH, RIGHT_SIDE_HIGH_PRELOAD, RIGHT_DIAGONAL,
        LEFT_DIAGONAL, RIGHT_SIDE_MID, RIGHT_SIDE_MID_PRELOAD,
    }

    public Turret(HardwareMap hardwareMap, boolean isAuto)
    {
        pidController= new BPIDFController(new PIDCoefficients(p, i, d), kV, kA, kStatic);

        turretMotor = hardwareMap.get(DcMotorEx.class, "hturret");

        encoder=new Encoder(hardwareMap.get(DcMotorEx.class, "hturret"));
        this.isAuto=isAuto;
        hallEffect = hardwareMap.get(AnalogInput.class, "hallEffect");
        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        webcam.setPipeline(detector = new Detector());
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                System.out.println("START");
            }
            public void onError(int errorCode) {
            }
        });
        dashboard.startCameraStream(webcam, 30);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        state=State.ZERO;
    }

    public void update() {
        updateTarget();

        //motorOil=controller.calculate(encoder.getCurrentPosition(), targetPos, time)/100;
        if(state!=State.MANUAL)
        {
            pidController.setTargetPosition(targetPos);

            turretMotor.setPower(pidController.update(encoder.getCurrentPosition()));
        }
    }

    public double secondsSpentInState() {
        return time.seconds();
    }
    public double millisecondsSpentInState() {
        return time.milliseconds();
    }

    private void updateTarget() {
        //if hall effect then reset pos at zero
        //if(true) {
            prevHall = hallEffect.getVoltage();
            if (hallEffect.getVoltage() - prevHall < -1.0) {
                if (turretMotor.getPower() < 0) {
                    posAtZero = -encoder.getCurrentPosition();
                } else {
                    posAtZero = -encoder.getCurrentPosition();
                }
            }
        //}

            switch (state) {
                case MANUAL:
                    targetPos = encoder.getCurrentPosition();
                    break;
                case IDLE:
                    targetPos = encoder.getCurrentPosition();
                    break;
                case RIGHT:
                    targetPos = RIGHT_POS - posAtZero;
                    break;
                case LEFT:
                    targetPos = LEFT_POS - posAtZero;
                    break;
                case BACK:
                    targetPos = BACK - posAtZero;
                    break;
                case ZERO:
                    targetPos = ZERO_POS - posAtZero;
                    break;
                case RIGHT_SIDE_HIGH:
                    targetPos = RIGHT_SIDE_HIGH - posAtZero;
                    break;
                case RIGHT_SIDE_HIGH_PRELOAD:
                    targetPos = RIGHT_SIDE_HIGH_PRELOAD - posAtZero;
                    break;
                case RIGHT_SIDE_MID:
                    targetPos = RIGHT_SIDE_MID - posAtZero;
                    break;
                case INIT:
                    targetPos = INIT - posAtZero;
                    break;
                case RIGHT_DIAGONAL:
                    targetPos = RIGHT_DIAGONAL - posAtZero;
                    break;
                case LEFT_DIAGONAL:
                    targetPos = LEFT_DIAGONAL - posAtZero;
                    break;
                case AUTOALIGN:
                    targetPos = encoder.getCurrentPosition() + detector.getShift();
                    break;
        }
    }

    public void resetPos() {
        posAtZero = encoder.getCurrentPosition();
    }
    public double getTargetPos()
    {
        return targetPos;
    }
    public boolean isBusy()
    {
        if(state==State.IDLE)
        {
            return false;
        }
        return true;
    }

    public Turret.State getState()
    {
        return state;
    }
}