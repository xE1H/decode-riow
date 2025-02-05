package org.firstinspires.ftc.teamcode.subsystems.hang;

import java.util.LinkedList;

public class DataBuffer {
    private final int BUFFER_SIZE = 2000;
    private LinkedList<Double> sensorValues;


    public DataBuffer() {
        sensorValues = new LinkedList<>();
    }


    public double getDelta(double newValue) {
        if (sensorValues.size() >= BUFFER_SIZE) sensorValues.removeFirst();

        sensorValues.add(newValue);

        if (sensorValues.size() < 2) return 0;
        return Math.abs(sensorValues.getLast() - sensorValues.getFirst());
    }


    public void resetList(){
        sensorValues.clear();
    }
}
