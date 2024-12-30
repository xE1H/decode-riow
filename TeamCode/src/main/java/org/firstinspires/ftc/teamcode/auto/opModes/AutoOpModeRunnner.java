package org.firstinspires.ftc.teamcode.auto.opModes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.commandFactory.CommandFactory;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;

@Photon
public class AutoOpModeRunnner {
    private Class<? extends VLRSubsystem<?>>[] requiredSubsystems;
    private boolean isInvertedMotors;
    private CommandFactory commandFactory;

    public AutoOpModeRunnner(CommandFactory commandFactory, boolean isInvertedMotors, Class<? extends VLRSubsystem<?>>... requiredSubsystems){
        this.commandFactory = commandFactory;
        this.isInvertedMotors = isInvertedMotors;
        this.requiredSubsystems = requiredSubsystems;
    }

    public void initialize(HardwareMap hardwareMap){
        VLRSubsystem.requireSubsystems(requiredSubsystems);
        VLRSubsystem.initializeAll(hardwareMap);
        GlobalConfig.setIsInvertedMotors(isInvertedMotors);

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(commandFactory.getStartingPoint(), 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(commandFactory.getStartingPoint());
        FollowPath.setFollower(follower);
    }

    public void run() {
        CommandScheduler.getInstance().schedule(commandFactory.getCommands());
    }
}
