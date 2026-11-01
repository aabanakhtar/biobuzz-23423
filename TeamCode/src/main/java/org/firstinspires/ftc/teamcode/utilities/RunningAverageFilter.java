package org.firstinspires.ftc.teamcode.utilities;

import java.util.Arrays;

public class RunningAverageFilter implements BasicFilter {

    private final double[] buffer;
    private final int windowSize;
    private int index = 0;
    private int count = 0;
    private double sum = 0.0;

    public RunningAverageFilter(int windowSize) {
        if (windowSize < 1) throw new IllegalArgumentException("windowSize must be >= 1");

        this.windowSize = windowSize;
        this.buffer = new double[windowSize];
    }

    @Override
    public void updateValue(double value) {
        if (count == windowSize) {
            sum -= buffer[index];
        } else {
            count++;
        }

        buffer[index] = value;
        sum += value;
        index = (index + 1) % windowSize;
    }

    @Override
    public double getFilteredOutput() {
        return (count == 0) ? 0.0 : sum / count;
    }

    @Override
    public void reset() {
        Arrays.fill(buffer, 0.0);
        index = 0;
        count = 0;
        sum = 0.0;
    }
}
