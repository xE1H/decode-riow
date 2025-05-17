package org.firstinspires.ftc.teamcode.controlmaps;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;


public abstract class ControlMap {
    DriverControls gp;

    public ControlMap(DriverControls driverControls) {
        gp = driverControls;
    }

    public abstract void initialize();
}
