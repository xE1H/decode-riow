package org.firstinspires.ftc.teamcode.subsystems.loader;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
public class Loader extends VLRSubsystem<Loader> {
    Servo loader;

    double LOADER_DOWN = 1;
    double LOADER_UP = 0.78;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        loader = hardwareMap.get(Servo.class, "loader");
        loader.setPosition(LOADER_DOWN);
    }

    public void up() {
        loader.setPosition(LOADER_UP);
    }

    public void down() {
        loader.setPosition(LOADER_DOWN);
    }
}
