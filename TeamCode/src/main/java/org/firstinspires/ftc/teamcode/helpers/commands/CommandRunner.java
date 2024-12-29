package org.firstinspires.ftc.teamcode.helpers.commands;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.CommandScheduler;

/**
 * Runnable class to run FTCLib command scheduler on an
 * independent thread from the main opmode loop.
 */
public class CommandRunner implements Runnable {
    final OpModeRunningInterface runningInterface;

    public CommandRunner(OpModeRunningInterface runningInterface) {
        this.runningInterface = runningInterface;
    }

    public void run() {
        while (!runningInterface.isOpModeRunning()) {
            try {
                sleep(10); // Wait for the opmode to start to start running commands
            } catch (InterruptedException e) {
                throw new RuntimeException(e); // some stupid shit so it compiles
            }
        }

        while (runningInterface.isOpModeRunning()) CommandScheduler.getInstance().run();
    }
}
