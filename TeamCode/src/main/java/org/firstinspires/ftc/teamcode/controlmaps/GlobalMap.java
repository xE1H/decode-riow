package org.firstinspires.ftc.teamcode.controlmaps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;

public class GlobalMap extends ControlMap {
    boolean slideResetActive = false;
    boolean rotatorResetActive = false;
    public boolean isBlue = false;

    public boolean followerActive = false;
    GamepadKeys.Button triangle = GamepadKeys.Button.A;
    public Follower f;
    public RumbleControls rc;

    public GlobalMap(DriverControls driverControls, Follower f, RumbleControls rc, boolean isBlue) {
        super(driverControls);
        this.f = f;
        this.rc = rc;
        this.isBlue = isBlue;
    }

    @Override
    public void initialize() {
        gp.add(new ButtonCtl(triangle, this::toggleFollower));
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
}
