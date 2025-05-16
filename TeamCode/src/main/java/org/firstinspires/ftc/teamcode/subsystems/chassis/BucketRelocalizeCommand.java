package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class BucketRelocalizeCommand extends SequentialCommandGroup {
    private final double velocityThreshold = 0;
    private final double maxAllowedDistanceDelta = 0;

    public BucketRelocalizeCommand(Follower follower){
        addCommands(
                new WaitUntilCommand(() -> follower.atParametricEnd() && follower.getVelocity().getMagnitude() < velocityThreshold),
                new InstantCommand() {
                    @Override
                    public void run() {
                        Pose calculatedPose = VLRSubsystem.getInstance(Chassis.class).calculateRobotPoseFromDistanceSensors(follower);
                        Pose currentPose = follower.getPose();

                        System.out.println("CALCULATED POSITION FROM DISTANCE SENSORS IS: " + calculatedPose.getX() + "; " + calculatedPose.getY() + "; " + Math.toDegrees(calculatedPose.getHeading()));
                        System.out.println("PEDRO ODOMETRY POSE IS: " + currentPose.getX() + "; " + currentPose.getY() + "; " + currentPose.getHeading());

                        double poseDeviation = Math.hypot(calculatedPose.getX() - currentPose.getX(), calculatedPose.getY() - currentPose.getY());

                        if (poseDeviation < maxAllowedDistanceDelta){
                            System.out.println("POSE DEVIATION IS ACCEPTABLE, RELOCALIZING");
                            follower.resetOffset();
                            follower.setCurrentPoseWithOffset(calculatedPose);
                        }

                        else{
                            System.out.println("POSE DEVIATION IS UNACCEPTABLE, SKIPPING RELOCALIZATION");

                        }
                    }
                }
        );
    }
}
