package org.firstinspires.ftc.teamcode.helpers.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.commands.CommandRunner;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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
    }

    public abstract void run();
}
