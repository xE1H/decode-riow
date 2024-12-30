package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

public class TelemetryDebugCommand extends InstantCommand {

    public TelemetryDebugCommand(Telemetry telemetry, String message){
        super(()-> telemetry.addLine(message));
    }

    public TelemetryDebugCommand(Telemetry telemetry){
        this(telemetry, "‼️‼️‼️‼️TELEMETRY DEBUG COMMAND‼️‼️‼️‼️‼️");
    }

    public TelemetryDebugCommand(Telemetry telemetry, String string, ArmState.State state){
        super(()-> telemetry.addData(string, state));
    }
}