package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

public abstract class ModuleTest extends LinearOpMode {
    private Module[] modules = {};
    private KeyReader[] keyReaders = {};
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public abstract void initialize();

    public abstract void update();

    public void setModules(Module... modules) {
        this.modules = modules;
    }

    public void setKeyReaders(KeyReader... keyReaders) {
        this.keyReaders = keyReaders;
    }

    public void opening() {}

    @Override
    public final void runOpMode() throws InterruptedException {
        initialize();
        for (Module module : modules) {
            module.init();
            module.setDebugMode(true);
        }
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        waitForStart();
        opening();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            for (KeyReader keyReader : keyReaders) {
                keyReader.readValue();
            }
            for (Module module : modules) {
                module.update();
            }
            update();
        }
    }
}
