package org.firstinspires.ftc.teamcode.helpers.monitoring;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Iterator;
import java.util.TreeSet;


public class LoopTimeMonitor {
    private final ElapsedTime et = new ElapsedTime();
    private boolean isLooping = false;
    private final TreeSet<Long> loopTimes = new TreeSet<>();

    public enum ElementSelectionType {
        TOP_N_ELEMENTS,
        TOP_PERCENTILE_ELEMENTS
    }
    
    public void loopStart() {
        et.reset();
        this.isLooping = true;
    }

    public void loopEnd() {
        if (!this.isLooping) {
            throw new IllegalStateException("Loop end called without loop start");
        }
        long loopTime = et.nanoseconds();
        this.loopTimes.add(loopTime);
        this.isLooping = false;
    }
    public double getAverageTime(int value){
        return getAverageTime(value, ElementSelectionType.TOP_N_ELEMENTS);
    }

    public double getAverageTime(int value, ElementSelectionType type) {
        if (this.loopTimes.isEmpty()) {
            return 0;
        }

        int elementCount;
        if (type == ElementSelectionType.TOP_N_ELEMENTS) {
            elementCount = Math.min(value, loopTimes.size());
        } else {
            elementCount = Math.max(1, (int) (loopTimes.size() * (value / 100.0)));
        }

        long totalTime = 0;
        Iterator<Long> iterator = loopTimes.descendingIterator();

        for(int i = 0; i < elementCount && iterator.hasNext(); i++){
            totalTime += iterator.next();
        }

        return totalTime / (double) elementCount / 1_000_000.0;
    }
    
}