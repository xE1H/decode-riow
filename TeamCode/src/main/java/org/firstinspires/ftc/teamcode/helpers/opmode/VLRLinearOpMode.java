package org.firstinspires.ftc.teamcode.helpers.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.helpers.monitoring.GlobalLoopTimeMonitor;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

public abstract class VLRLinearOpMode extends LinearOpMode {
    // Execution
    ExecutorService executorService;
    // Commands
    CommandRunner commandRunner;

    Logger logger = Logger.getLogger("VLRLinearOpMode");

    double startTime = 0;

    Runnable beforeEndRunnable;

    @Override
    public void runOpMode() {
        VLRSubsystem.clearSubsystems(); // Clear all subsystems
        executorService = Executors.newCachedThreadPool();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        commandRunner = new CommandRunner(this::opModeIsActive, hardwareMap, telemetry);
        executorService.submit(commandRunner);

        this.run();
        GlobalLoopTimeMonitor.shutDown();

        CommandScheduler.getInstance().reset(); // reset command scheduler -- clear all previous commands
        if (beforeEndRunnable != null) beforeEndRunnable.run();
        executorService.shutdownNow(); // force shutdown of ALL threads
        try {
            //noinspection ResultOfMethodCallIgnored
            executorService.awaitTermination(100, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            logger.warning("Force shutdown of all threads timed out");
        }
        VLRSubsystem.clearSubsystems(); // Clear all subsystems
    }

    protected void setBeforeEndRunnable(Runnable beforeEndRunnable) {
        this.beforeEndRunnable = beforeEndRunnable;
    }

    public abstract void run();

    @Override
    public void waitForStart() {
        super.waitForStart();
        startTime = super.getRuntime();
    }

    /**
     * @return the time in seconds since the opmode was initialized
     */
    public double getTimeSinceInit() {
        return super.getRuntime() - startTime;
    }

    /**
     * @return the time in seconds since the opmode was started
     */
    public double getTimeSinceStart() {
        return super.getRuntime();
    }
}
