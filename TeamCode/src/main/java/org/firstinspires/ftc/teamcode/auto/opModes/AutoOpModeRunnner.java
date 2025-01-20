package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.commands.factory.CommandFactory;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;


@Photon
public class AutoOpModeRunnner {
    private boolean isInvertedMotors;
    private CommandFactory commandFactory;
    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    private Follower follower;


    public AutoOpModeRunnner(CommandFactory commandFactory, boolean isInvertedMotors) {
        this.commandFactory = commandFactory;
        this.isInvertedMotors = isInvertedMotors;
    }

    public void initialize(HardwareMap hardwareMap) {
        VLRSubsystem.requireSubsystems(
                commandFactory.getRequiredSubsystems()
        );
        VLRSubsystem.initializeAll(hardwareMap);
        GlobalConfig.setIsInvertedMotors(isInvertedMotors);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(commandFactory.getStartingPoint(), 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(commandFactory.getStartingPoint());
        FollowPath.setFollower(follower);

        CommandScheduler.getInstance().schedule(commandFactory.getCommands());
    }

    public void run(BooleanSupplier isActive, boolean enableDebug) {
        while (isActive.getAsBoolean()) {
            follower.update();
            if (enableDebug) {
                follower.telemetryDebug(telemetry);
            }
        }
    }

    public void run(BooleanSupplier isActive){
        run(isActive, true);
    }
}
