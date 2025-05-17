package org.firstinspires.ftc.teamcode.helpers.utils.opmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
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

    ElapsedTime measurementTimer = new ElapsedTime();


    @Override
    public void Start(){
    }

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
    public void Loop() {
//        if (extension != prevExtension){
//            prevExtension = extension;
//            CommandScheduler.getInstance().schedule(new SetArmPosition().extension(extension));
//        }
//
//        if (angle != prevAngle){
//            prevAngle = angle;
//            CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(angle));
//        }

        if (gamepad1.triangle && !prevTriangle){
            follower.setMaxPower(0);
            CommandScheduler.getInstance().schedule(new SetArmPosition().level_3_hang(()-> gamepad1.right_bumper && gamepad1.left_bumper));
        }

//        if (gamepad1.cross && !prevCross){
//            VLRSubsystem.getArm().setThirdSlideMotorEnable(false);
//            VLRSubsystem.getArm().enableRotatorPowerOverride(0);
//        }
//
//        if (!gamepad1.cross && prevCross){
//            VLRSubsystem.getArm().setThirdSlideMotorEnable(true);
//            VLRSubsystem.getArm().disableRotatorPowerOverride();
//        }

        if (gamepad1.circle && !prevCircle){
            overrideEnabled = true;
        }

        if (overrideEnabled){
            VLRSubsystem.getArm().enableSlidePowerOverride(gamepad1.left_stick_y);
            VLRSubsystem.getArm().enableRotatorPowerOverride(gamepad1.right_stick_y);
        }


//        else if (gamepad1.cross && !prevCross){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().retract());
//        }
//
//        else if (gamepad1.square && !prevSquare){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().intakeSample(0.34));
//        }

//        if (gamepad1.triangle && !prevTriangle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().extension(1));
//        }
//
//        else if (gamepad1.cross && !prevCross){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().extension(0));
//        }
//
//        else if (gamepad1.square && !prevSquare){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(90));
//        }
//
//        else if (gamepad1.circle && !prevCircle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(0));
//        }


//        if (gamepad1.triangle && !prevTriangle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().XY(40, 30, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
//        }
//
//        else if (gamepad1.cross && !prevCross){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().XY(10, 0, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
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