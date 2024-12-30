package org.firstinspires.ftc.teamcode.auto.commandFactory;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class ObservationCommandFactory extends CommandFactory {
    private boolean isBlueTeam;

    private Point startingPoint;
    private Point toSpecimenScore;
    /**
     * Not sure if this point is necessary
     */
    private Point rotate;
    private Point toAllSamplesControl1;
    private Point toAllSamplesControl2;
    private Point toAllSamples;
    private Point toSample1Horizontal;
    private Point sample1ToObservation;
    private Point toSample1Vertical;
    private Point toSample2Horizontal;
    private Point sample2ToObservation;
    private Point toSample2Vertical;
    private Point toSample3Horizontal;
    private Point sample3ToObservation;


    public ObservationCommandFactory(boolean isBlueTeam) {
        initializePointsForBlueTeam();
        this.isBlueTeam = isBlueTeam;
        if (!isBlueTeam) {
            Point[] allPoints = {startingPoint, toSpecimenScore, rotate, toAllSamplesControl1, toAllSamplesControl2, toAllSamples, toSample1Horizontal, sample1ToObservation, toSample1Vertical, toSample2Horizontal, sample2ToObservation, toSample2Vertical, toSample3Horizontal, sample3ToObservation};
            mirrorPointsToRedTeam(
                    allPoints
            );
        }
    }

    @Override
    public void initializePointsForBlueTeam() {
        startingPoint = new Point(9.6, 60);
        toSpecimenScore = new Point(28, 60);
        rotate = new Point(28.05, 60);
        toAllSamplesControl1 = new Point(29, 42.5);
        toAllSamplesControl2 = new Point(38.5, 29);
        toAllSamples = new Point(57.5, 29);
        toSample1Horizontal = new Point(57.5, 19);
        sample1ToObservation = new Point(21, 19);
        toSample1Vertical = new Point(57.5, 19);
        toSample2Horizontal = new Point(57, 9.5);
        sample2ToObservation = new Point(21, 9.5);
        toSample2Vertical = new Point(57, 9.5);
        toSample3Horizontal = new Point(57, 3.5);
        sample3ToObservation = new Point(21, 3.5);
    }
    @Override
    public Point getStartingPoint() {
        return startingPoint;
    }
    @Override
    public SequentialCommandGroup getCommands() {
        return new SequentialCommandGroup(
                new FollowPath(0, toSpecimenScore),
                new FollowPath(0, -90, rotate),
                new WaitCommand(500),
                new FollowPath(!isBlueTeam, toAllSamplesControl1, toAllSamplesControl2, toAllSamples),
                new FollowPath(0, toSample1Horizontal),
                new FollowPath(0, sample1ToObservation),
                new FollowPath(0, toSample1Vertical),
                new FollowPath(0, toSample2Horizontal),
                new FollowPath(0, sample2ToObservation),
                new FollowPath(0, toSample2Vertical),
                new FollowPath(0, toSample3Horizontal),
                new FollowPath(0, sample3ToObservation)
        );

    }

}
