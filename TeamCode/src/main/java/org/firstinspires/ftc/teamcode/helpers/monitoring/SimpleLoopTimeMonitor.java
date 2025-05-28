package org.firstinspires.ftc.teamcode.helpers.monitoring;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class SimpleLoopTimeMonitor {
    private static final AtomicLong commandStartTime = new AtomicLong(0);
    private static final AtomicLong mainStartTime = new AtomicLong(0);

    private static final AtomicBoolean shutdownRequested = new AtomicBoolean(false);

    private static final AtomicReference<Double> commandLoopHz = new AtomicReference<>(0.0);
    private static final AtomicReference<Double> mainLoopHz = new AtomicReference<>(0.0);

    public static void shutDown(){
        shutdownRequested.set(true);
    }

    // Call at the start of the command loop
    public static void commandLoopStart() {
        commandStartTime.set(System.nanoTime());
    }

    // Call at the end of the command loop
    public static void commandLoopEnd() {
        long durationNs = System.nanoTime() - commandStartTime.get();
        if (durationNs > 0) {
            double hz = 1_000_000_000.0 / durationNs;
            commandLoopHz.set(hz);
        }
    }

    // Call at the start of the main loop
    public static void mainLoopStart() {
        mainStartTime.set(System.nanoTime());
    }

    // Call at the end of the main loop
    public static void mainLoopEnd() {
        long durationNs = System.nanoTime() - mainStartTime.get();
        if (durationNs > 0) {
            double hz = 1_000_000_000.0 / durationNs;
            mainLoopHz.set(hz);
        }
    }

    // Call to log the most recent frequencies
    public static void logLoopTimes(Telemetry telemetry) {
        if (!shutdownRequested.get()) {
            telemetry.addData("Main Loop Frequency (Hz)", String.format("%.3f", mainLoopHz.get()));
            telemetry.addData("Command Loop Frequency (Hz)", String.format("%.3f", commandLoopHz.get()));
        }
    }
}