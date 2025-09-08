package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.controlmaps.GlobalMap;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.persistence.AllianceSaver;
import org.firstinspires.ftc.teamcode.helpers.persistence.PoseSaver;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;

@TeleOp(name = "VLRTeleOp", group = "!TELEOP")
@Photon
public class VLRTeleOp extends VLRLinearOpMode {
    static Pose NO_AUTO_START_POSE = new Pose(0,0,0);

    Follower f;
    CommandScheduler cs;
    RumbleControls rc;

    boolean isBlue = true;
    boolean prevFollowerActive = true;
    @Override
    public void run() {
        cs = CommandScheduler.getInstance();
        f = new Follower(hardwareMap, FConstants.class, LConstants.class);

        //noinspection unchecked
        VLRSubsystem.requireSubsystems();
        VLRSubsystem.initializeAll(hardwareMap);

        if (!PoseSaver.isPoseSaved()) {
            telemetry.addLine("No saved pose found, using default");
            // Set default sample start pos
            f.setStartingPose(NO_AUTO_START_POSE);
        } else {
            telemetry.addData("Saved pose: ", PoseSaver.getPedroPose());
            f.setStartingPose(PoseSaver.getPedroPose());
        }

        if (AllianceSaver.getAlliance() == null) {
            telemetry.addLine("No saved alliance found, defaulting to Blue");
        } else if (AllianceSaver.getAlliance() == Alliance.RED) {
            isBlue = false;
        }
        telemetry.update();


        GamepadEx gpEx = new GamepadEx(gamepad1);
        DriverControls gp = new DriverControls(gpEx);
        rc = new RumbleControls(gamepad1);

        GlobalMap globalMap = new GlobalMap(gp, f, rc, isBlue);
        globalMap.initialize();

        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (opModeIsActive()) {
            gp.update();

            if (globalMap.followerActive) f.update();
            else {
                if (!prevFollowerActive){
                    VLRSubsystem.getInstance(Chassis.class).setMotorsToBrake();
                }

                // Not defining these controls through DriverControls cuz ts pmo
                VLRSubsystem.getInstance(Chassis.class).drive(gpEx.getLeftY(), -gpEx.getLeftX(), -0.3 * gpEx.getRightX());
                f.updatePose();
            }
            prevFollowerActive = globalMap.followerActive;

            telemetry.update();

            Pose currentPose = f.getPose();
            if (!currentPose.roughlyEquals(new Pose(0,0,0))) {
                PoseSaver.setPedroPose(currentPose);
            }
        }
    }
}
