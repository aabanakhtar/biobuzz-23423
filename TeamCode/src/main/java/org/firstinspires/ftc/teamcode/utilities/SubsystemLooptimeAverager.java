package org.firstinspires.ftc.teamcode.utilities;

public class SubsystemLooptimeAverager
{
    private long startNs = 0;

    private double avgMs = 0.0;
    private long samples = 0;

    public void mark() {
        startNs = System.nanoTime();
    }

    public void endMark() {
        long dtNs = System.nanoTime() - startNs;
        double dtMs = dtNs / 1e6;

        samples++;

        // cumulative running average (gets flatter over time)
        avgMs += (dtMs - avgMs) / samples;
    }

    public double getAvgMs() {
        return avgMs;
    }
}
