package org.firstinspires.ftc.teamcode.helpers.testOpmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

@Config
@Photon
@TeleOp(group = "Utils")

public class ArmSoftLimitDemonstration extends VLRLinearOpMode {
    public boolean prevDpadUp;
    public boolean prevDpadDown;
    private Follower follower;

    @Override
    public void run() {
        FConstants.initialize();

        ///VERY CRITICAL follower must be before subsystem init cause it reveres arm motors
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose());

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, Chassis.class, ClawSubsystem.class, HangSubsystem.class, BlinkinSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.dpad_up && !prevDpadUp){
                CommandScheduler.getInstance().schedule(new SetArmPosition().retract().andThen(new WaitCommand(1000), new SetArmPosition().intakeSample(0.44)));
            }
            if (gamepad1.dpad_down && !prevDpadDown){
                CommandScheduler.getInstance().schedule(
                        new SetArmPosition().retract().andThen(new WaitCommand(1000), new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET)));
            }

            prevDpadDown = gamepad1.dpad_down;
            prevDpadUp = gamepad1.dpad_up;

            telemetry.update();
        }
    }
}