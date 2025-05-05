package org.firstinspires.ftc.teamcode.teleop.controlmaps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;

import java.util.logging.Logger;

public class GlobalMap extends ControlMap {
    boolean slideResetActive = false;
    boolean rotatorResetActive = false;

    public boolean followerActive = false;

    public LimelightYoloReader reader = new LimelightYoloReader();

    public Follower f;
    public RumbleControls rc;

    public GlobalMap(DriverControls driverControls, Follower f, RumbleControls rc) {
        super(driverControls);
    }

    @Override
    public void initialize() {
        gp.add(new ButtonCtl(GamepadKeys.Button.A, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> toggleFollower()));

        gp.add(new ButtonCtl(GamepadKeys.Button.X, ButtonCtl.Trigger.STATE_JUST_CHANGED, (Boolean a) -> {
            if (!slideResetActive) startSlideOverride();
            else endSlideOverride();
        }));
        gp.add(new ButtonCtl(GamepadKeys.Button.Y, ButtonCtl.Trigger.STATE_JUST_CHANGED, (Boolean a) -> {
            if (!rotatorResetActive) startRotatorOverride();
            else endRotatorOverride();
        }));
    }

    //
    // FOLLOWER
    //
    private void toggleFollower() {
        followerActive = !followerActive;

        if (followerActive) {
            rc.doubleBlip();
            f.holdPoint(f.getPose());
        } else {
            rc.singleBlip();
        }
    }

    //
    // UTILS
    //
    private void wipe(double x) {
        // x here should be 0-1
        VLRSubsystem.getInstance(Wiper.class).wipe(x);
    }

    private void startSlideOverride() {
        Logger.getLogger("SlideOverride").fine("Start override");
        slideResetActive = true;
        VLRSubsystem.getArm().enableSlidePowerOverride(-0.3);
    }

    private void endSlideOverride() {
        Logger.getLogger("SlideOverride").fine("End override");
        slideResetActive = false;
        VLRSubsystem.getArm().disableSlidePowerOverride();
    }

    private void startRotatorOverride() {
        Logger.getLogger("RotatorOverride").fine("Start override");
        rotatorResetActive = true;
        VLRSubsystem.getArm().enableRotatorPowerOverride(-0.1);
    }

    private void endRotatorOverride() {
        Logger.getLogger("RotatorOverride").fine("End override");
        rotatorResetActive = false;
        VLRSubsystem.getArm().disableRotatorPowerOverride();
    }
}
