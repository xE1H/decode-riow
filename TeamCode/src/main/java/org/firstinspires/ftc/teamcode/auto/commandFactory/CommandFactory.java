package org.firstinspires.ftc.teamcode.auto.commandFactory;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public abstract class CommandFactory {
    protected void mirrorPointsToRedTeam(Point[] allPoints){
        for (Point point : allPoints) {
            point.mirrorCartesianX();
            point.mirrorCartesianY();
        }
    }
    public abstract void initializePointsForBlueTeam();
    public abstract Point getStartingPoint();
    public abstract SequentialCommandGroup getCommands();
}
