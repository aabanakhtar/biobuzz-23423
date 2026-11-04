package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Ashwing;

import java.util.List;

public class Hubs {
    private final List<LynxModule> modules;
    private final ElapsedTime timer;
    double dt = 0;

    public Hubs() {
        modules = Ashwing.get().lynxModules;
        timer = new ElapsedTime();
        clearCache();
    }

    public void clearCache() {
        modules.forEach(LynxModule::clearBulkCache);

        dt = timer.seconds();
        timer.reset();
    }

    public double getDeltaTime() {
        return dt;
    }
}
