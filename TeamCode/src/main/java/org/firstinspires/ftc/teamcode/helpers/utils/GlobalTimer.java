package org.firstinspires.ftc.teamcode.helpers.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;

import java.util.logging.Level;

public class GlobalTimer {
    private static double elapsedTime = 0;
    private static boolean stopped = false;
    private static double startTime = System.nanoTime();

    public static double time(){
        if (stopped) return elapsedTime;
        return elapsedTime + (System.nanoTime() - startTime) / Math.pow(10, 9);
    }


    public static void resetTimer(){
        startTime = System.nanoTime();
        elapsedTime = 0;
    }


    public static Command resetCommand() {
        return new CommandBase() {
            boolean hasRun = false;

            @Override
            public void execute(){
                if (!hasRun) {
                    startTime = System.nanoTime();
                    elapsedTime = 0;
                    hasRun = true;
                }
            }

            @Override
            public boolean isFinished(){
                return hasRun;
            }
        };
    }

    public static Command stopTimer(){
        return new InstantCommand() {
            @Override
            public void run() {
                elapsedTime = time();
                stopped = true;
            }
        };
    }

    public static Command resumeTimer(){
        return new InstantCommand() {
            @Override
            public void run() {
                stopped = false;
                startTime = System.nanoTime();
            }
        };
    }

    public static Command logTime(){
        return new LogCommand("GLOBAL TIMER", Level.INFO, ()-> "GLOBAL TIMER TIME: " + time());
    }

    public static Command logTime(String message){
        return new LogCommand("GLOBAL TIMER", Level.INFO, ()-> message + time());
    }

    public static Command logAutoTime(){
        return new LogCommand("GLOBAL TIMER", Level.INFO, ()-> "FINAL AUTO TIME: " + time());
    }
}