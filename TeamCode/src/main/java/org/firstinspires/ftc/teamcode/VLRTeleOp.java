package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.START_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB_0;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrab;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.controls.trigger.TriggerCtl;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.persistence.PoseSaver;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;
import org.firstinspires.ftc.teamcode.teleop.controlmaps.GlobalMap;
import org.firstinspires.ftc.teamcode.teleop.controlmaps.SampleMap;
import org.firstinspires.ftc.teamcode.teleop.controlmaps.SpecimenMap;

import java.sql.Driver;
import java.util.logging.Logger;

import pedroPathing.tuners.constants.FConstants;
import pedroPathing.tuners.constants.LConstants;

@TeleOp(name = "VLRTeleOp", group = "!TELEOP")
@Photon
public class VLRTeleOp extends VLRLinearOpMode {
    Follower f;
    CommandScheduler cs;
    RumbleControls rc;

    boolean sampleMapActive = true;


    @Override
    public void run() {
        cs = CommandScheduler.getInstance();
        f = new Follower(hardwareMap, FConstants.class, LConstants.class);

        //noinspection unchecked
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class, Wiper.class);
        VLRSubsystem.initializeAll(hardwareMap);

        if (!PoseSaver.isPoseSaved()) {
            // Set default sample start pos
            f.setStartingPose(START_POSE);
            telemetry.addLine("No saved pose found, using default");
            telemetry.update();
        } else {
            telemetry.addData("Saved pose: ", PoseSaver.getPedroPose());
            telemetry.update();
            f.setStartingPose(PoseSaver.getPedroPose());
        }

        GamepadEx gpEx = new GamepadEx(gamepad1);
        DriverControls gp = new DriverControls(gpEx);
        rc = new RumbleControls(gamepad1);

        // GlobalMap is for general controls that are not specific to samples/specimen
        GlobalMap globalMap = new GlobalMap(gp, f, rc);
        globalMap.initialize();

        SampleMap sampleMap = new SampleMap(gp, cs, globalMap);
        sampleMap.initialize(); // initialize sample controls as default

        SpecimenMap specimenMap = new SpecimenMap(gp, cs, globalMap);

        // Add switch control for swithing between the maps
        addSwitchCtrl(gp, globalMap, sampleMap, specimenMap);

        waitForStart();
        cs.schedule(new SetArmPosition().retractAfterAuto());

        while (opModeIsActive()) {
            gp.update();

            if (globalMap.followerActive) f.update();
            else {
                // Not defining these controls through DriverControls cuz ts pmo
                VLRSubsystem.getInstance(Chassis.class).drive(gpEx.getLeftY(), -gpEx.getLeftX(), -0.3 * gpEx.getRightX());
                f.updatePose();
            }
        }
    }

    private void addSwitchCtrl(DriverControls gp, GlobalMap globalMap, SampleMap sampleMap, SpecimenMap specimenMap) {
        gp.add(new ButtonCtl(GamepadKeys.Button.LEFT_STICK_BUTTON, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> {
            gp.clear();
            globalMap.initialize();
            if (sampleMapActive) {
                sampleMapActive = false;
                specimenMap.initialize();
                rc.rumbleBlips(6);
            } else {
                sampleMapActive = true;
                sampleMap.initialize();
                rc.rumbleBlips(3);
            }
            addSwitchCtrl(gp, globalMap, sampleMap, specimenMap);
        }));
    }
}
