package org.firstinspires.ftc.teamcode.helpers.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public abstract class VLRLinearOpMode extends LinearOpMode {
    // Execution
    ExecutorService executorService;
    // Commands
    CommandRunner commandRunner;

    @Override
    public void runOpMode() {
        executorService = Executors.newCachedThreadPool();

        commandRunner = new CommandRunner(this::opModeIsActive);
        executorService.submit(commandRunner);

        this.run();
        CommandScheduler.getInstance().reset(); // reset command scheduler -- clear all previous commands
        executorService.shutdownNow(); // force shutdown of ALL threads
        try {
            executorService.awaitTermination(100, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            System.out.println("Force shutdown of all threads timed out");
        }
        VLRSubsystem.clearSubsystems(); // Clear all subsystems
    }

    public abstract void run();
}
