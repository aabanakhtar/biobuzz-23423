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
        if (dt > 0) {
            // send info to ftc dash and phone
            Ashwing.get().flightRecorder.addLine("=====SYSTEM=======");
            Ashwing.get().flightRecorder.addData("Loop Time, (HZ)", 1.0 / dt);
        }

        timer.reset();
    }

    public double getDeltaTime() {
        return dt;
    }
}
