package org.firstinspires.ftc.teamcode.device;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AbsoluteAnalogEncoder {
    private final AnalogInput encoder;
    public double offset;

    public AbsoluteAnalogEncoder(HardwareMap map, String id, double off) {
        encoder = map.get(AnalogInput.class, id);
        offset = off;
    }

    public double getCurrentPosition() {
        return AngleUnit.normalizeDegrees(getVoltage() / 3.24 * 360 - offset);
    }

    public double getCurrentPositionNoOffset() {
        return (getVoltage()) / (3.24) * 360;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }
}
