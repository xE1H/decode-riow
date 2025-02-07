package org.firstinspires.ftc.teamcode.helpers.commands;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.monitoring.LoopTimeMonitor;

import java.util.List;

/**
 * Runnable class to run FTCLib command scheduler on an
 * independent thread from the main opmode loop.
 */
public class CommandRunner implements Runnable {
    final OpModeRunningInterface runningInterface;
    private HardwareMap hardwareMap;
    private LoopTimeMonitor loopTimeMonitor = new LoopTimeMonitor();

    public CommandRunner(OpModeRunningInterface runningInterface, HardwareMap hardwareMap) {
        this.runningInterface = runningInterface;
        this.hardwareMap = hardwareMap;
    }

    public void run() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        while (!runningInterface.isOpModeRunning()) {
            try {
                sleep(10); // Wait for the opmode to start to start running commands
            } catch (InterruptedException e) {
                throw new RuntimeException(e); // some stupid shit so it compiles
            }
        }

        while (runningInterface.isOpModeRunning()){
            loopTimeMonitor.loopStart();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            loopTimeMonitor.loopEnd();

            double cycleTime = loopTimeMonitor.getAverageTime(5, LoopTimeMonitor.ElementSelectionType.TOP_PERCENTILE_ELEMENTS) / 1000;
            System.out.println("COMMAND THREAD CYCLE TIME: " + 1.0 / cycleTime);
        }
    }
}
