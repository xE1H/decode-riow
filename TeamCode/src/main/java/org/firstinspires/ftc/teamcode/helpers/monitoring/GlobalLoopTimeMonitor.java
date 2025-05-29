package org.firstinspires.ftc.teamcode.helpers.monitoring;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.atomic.AtomicLong;

public class GlobalLoopTimeMonitor {
    // Thread-safe storage for the last calculated frequencies
    private static volatile AtomicLong mainLoopDurationNs = new AtomicLong(0);

    private static volatile long commandThreadDurationNs = 0;

    private static volatile boolean shutdownRequested = false;

    public static void shutDown() {
        shutdownRequested = true;
    }

    public static void reset() {
        shutdownRequested = false;
    }

    public static void setMainLoopDurationNs(long durationNs) {
        mainLoopDurationNs.set(durationNs);
    }

    public static void setCommandLoopDurationNs(long durationNs) {
        commandThreadDurationNs = durationNs;
    }


    public static void logLoopTimes(Telemetry telemetry) {
        if (shutdownRequested || telemetry == null) return;

        double mainThreadFrequency = 1_000_000_000d / mainLoopDurationNs.longValue();
        double commandThreadFrequency = 1_000_000_000d / commandThreadDurationNs;

        try {
            telemetry.addData("Main thread", "%.1f hz", mainThreadFrequency);
            telemetry.addData("Command thread", "%.1f hz", commandThreadFrequency);
        } catch (Exception e) {
            System.out.println(e);
        }
    }
}