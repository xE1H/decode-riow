package org.firstinspires.ftc.teamcode.helpers.testOpmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRTestOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

@Config
@Photon
@TeleOp()
public class ArmCommandMagnitudeAndExtensionTest extends VLRTestOpMode {
    public static double extension = 0;
    public static double angle = 0;

    private double prevExtension = 0;
    private double prevAngle = 0;

    boolean prevTriangle = false;
    boolean prevCross = false;
    boolean prevSquare = false;
    boolean prevCircle = false;

    boolean overrideEnabled = false;

    Follower follower;
    GamepadEx gamepad;
    RumbleControls rc;

    boolean analogEnabled = false;

    @Override
    public void Init(){
        FConstants.initialize();

        ///VERY CRITICAL follower must be before subsystem init cause it reveres arm motors
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose());

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, Chassis.class, ClawSubsystem.class, HangSubsystem.class, BlinkinSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        gamepad = new GamepadEx(gamepad1);
        rc = new RumbleControls(gamepad1);

        ArmState.set(ArmState.State.IN_ROBOT);
    }

    @Override
    public void Start(){
        //CommandScheduler.getInstance().schedule(new SetArmPosition().level_3_hang(gamepad1, ()-> analogEnabled));

        CommandScheduler.getInstance().schedule(new SetClawState(ClawConfiguration.GripperState.CLOSED));
        CommandScheduler.getInstance().schedule(new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL));
        CommandScheduler.getInstance().schedule(new SetClawAngle(ClawConfiguration.VerticalRotation.UP));
    }


    @Override
    public void Loop() {
        if (gamepad1.triangle && !prevTriangle){
            analogEnabled = true;
        }

        if (analogEnabled){
            VLRSubsystem.getArm().enableSlidePowerOverride(-0.5 + gamepad1.right_stick_y);
            VLRSubsystem.getArm().enableRotatorPowerOverride(-0.35 + gamepad1.left_stick_y);
        }

        if (gamepad1.circle && !prevCircle){
            analogEnabled = false;
            VLRSubsystem.getArm().disableSlidePowerOverride();
            VLRSubsystem.getArm().disableRotatorPowerOverride();

            CommandScheduler.getInstance().schedule(new SetArmPosition().extensionAndAngleDegreesNOTSAFEJUSTFORHANG(0, 50));
        }


//        if (gamepad1.triangle && !prevTriangle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().scoreSpecimenBack());
//        }
//
//        else if (gamepad1.cross && !prevCross){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().intakeSpecimen(0.4));
//        }

        prevTriangle = gamepad1.triangle;
        prevCross = gamepad1.cross;
        prevSquare = gamepad1.square;
        prevCircle = gamepad1.circle;

        follower.update();
        VLRSubsystem.getInstance(Chassis.class).drive(0, 0, 0);
        telemetry.update();
    }

}