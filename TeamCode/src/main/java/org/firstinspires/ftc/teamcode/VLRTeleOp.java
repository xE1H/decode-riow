package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.START_POSE;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.RED;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.YELLOW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.monitoring.GlobalLoopTimeMonitor;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.controlmaps.GlobalMap;
import org.firstinspires.ftc.teamcode.controlmaps.SampleMap;
import org.firstinspires.ftc.teamcode.controlmaps.SpecimenMap;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.persistence.AllianceSaver;
import org.firstinspires.ftc.teamcode.helpers.persistence.PoseSaver;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;


import java.util.Arrays;
import java.util.Collections;

@TeleOp(name = "VLRTeleOp", group = "!TELEOP")
@Photon
public class VLRTeleOp extends VLRLinearOpMode {
    Follower f;
    CommandScheduler cs;
    RumbleControls rc;

    boolean sampleMapActive = true;
    boolean isBlue = true;
    boolean analogHangActive = false;
    boolean prevTriangle = false;
    boolean hangHoldPosition = false;
    boolean prevDpad = false;
    boolean prevFollowerActive = false;


    @Override
    public void run() {
        GlobalConfig.DEBUG_MODE = true;

        cs = CommandScheduler.getInstance();
        f = new Follower(hardwareMap, FConstants.class, LConstants.class);

        //noinspection unchecked
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class, Wiper.class, BlinkinSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        if (!PoseSaver.isPoseSaved()) {
            telemetry.addLine("No saved pose found, using default");
            // Set default sample start pos
            f.setStartingPose(START_POSE);
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

        // GlobalMap is for general controls that are not specific to samples/specimen
        GlobalMap globalMap = new GlobalMap(gp, f, rc);
        globalMap.initialize();

        SampleMap sampleMap = new SampleMap(gp, cs, globalMap);
        sampleMap.initialize(); // initialize sample controls as default
        globalMap.reader.setAllowedColors(Arrays.asList(BLUE, YELLOW));

        SpecimenMap specimenMap = new SpecimenMap(gp, cs, globalMap);

        // Add switch control for swithing between the maps
        addSwitchCtrl(gp, globalMap, sampleMap, specimenMap);



        waitForStart();
        cs.schedule(new SetArmPosition().retractAfterAuto());
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GlobalConfig.DEBUG_MODE = true;

        while (opModeIsActive()) {
            gp.update();

//            if (analogHangActive){
//                VLRSubsystem.getArm().enableSlidePowerOverride(-0.5 + gamepad2.right_stick_y);
//                VLRSubsystem.getArm().enableRotatorPowerOverride(-0.35 + gamepad2.left_stick_y);
//            }

//            if (gamepad1.triangle && !prevTriangle && !hangHoldPosition){
//                analogHangActive = true;
//            }

//            if (gamepad2.right_trigger > 0.9 && !hangHoldPosition){
//                analogHangActive = false;
//                VLRSubsystem.getArm().disableSlidePowerOverride();
//                VLRSubsystem.getArm().disableRotatorPowerOverride();
//                CommandScheduler.getInstance().schedule(new SetArmPosition().extensionAndAngleDegreesNOTSAFEJUSTFORHANG(0, 50));
//            }


            if (gamepad2.dpad_down && !prevDpad){
                prevDpad = true;
                CommandScheduler.getInstance().schedule(
                        new SetArmPosition().level_3_hang(()-> gamepad1.dpad_right, ()-> false)
//                                .alongWith(
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(()-> analogHangActive),
//                                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() < 0.05 && VLRSubsystem.getArm().currentAngleDegrees() < 65),
//                                        new InstantCommand(()-> analogHangActive = false),
//                                        new InstantCommand(()-> VLRSubsystem.getArm().disableRotatorPowerOverride()),
//                                        new InstantCommand(()-> VLRSubsystem.getArm().disableSlidePowerOverride()),
//                                        new SetArmPosition().extensionAndAngleDegreesNOTSAFEJUSTFORHANG(0, 50)
//                                )
//                        )
                );
            }

            prevTriangle = gamepad1.triangle;


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
        }
    }

    private void addSwitchCtrl(DriverControls gp, GlobalMap globalMap, SampleMap sampleMap, SpecimenMap specimenMap) {
        gp.add(new ButtonCtl(GamepadKeys.Button.LEFT_STICK_BUTTON, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> {
            gp.clear();
            globalMap.initialize();
            if (sampleMapActive) {
                if (isBlue) {
                    globalMap.setLimelightColors(Collections.singletonList(BLUE));
                } else {
                    globalMap.setLimelightColors(Collections.singletonList(RED));
                }
                sampleMapActive = false;
                specimenMap.initialize();
                rc.rumbleBlips(4);
            } else {
                if (isBlue) {
                    globalMap.setLimelightColors(Arrays.asList(BLUE, YELLOW));
                } else {
                    globalMap.setLimelightColors(Arrays.asList(RED, YELLOW));
                }
                sampleMapActive = true;
                sampleMap.initialize();
                rc.rumbleBlips(2);
            }
            addSwitchCtrl(gp, globalMap, sampleMap, specimenMap);
        }));
    }
}
