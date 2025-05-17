package org.firstinspires.ftc.teamcode.helpers.opmode;

import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.RED;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.YELLOW;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.persistence.AllianceSaver;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.helpers.persistence.PoseSaver;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.Pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.Pedro.constants.LConstants;


public abstract class VLRAutoTestOpMode extends VLRLinearOpMode {
    CommandScheduler cs;
    Follower f;
    ElapsedTime autoTimer = new ElapsedTime();

    Command autoCommand;
    boolean autoFinished = false;
    boolean prevAutoFinished = false;
    double autoTime = 0;

    private AutoConfigurator ac;

    @Override
    public void run() {
        cs = CommandScheduler.getInstance();
        FConstants.initialize();

        f = new Follower(hardwareMap, FConstants.class, LConstants.class);

        //noinspection unchecked
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, BlinkinSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LimelightYoloReader reader = new LimelightYoloReader();

        ac = new AutoConfigurator(telemetry, gamepad1);
        AutoConfigurator.Choice color = ac.multipleChoice("Select alliance:", new AutoConfigurator.Choice("Blue"),
                new AutoConfigurator.Choice("Red"));

        boolean isBlue = color.text.equals("Blue");

        List<LimelightYoloReader.Limelight.Sample.Color> allowedColors = new ArrayList<>();

        if (isBlue) {allowedColors.add(BLUE);}
        else {allowedColors.add(RED);}
        if (!SpecimenOnly()) {allowedColors.add(YELLOW);}

        reader.setAllowedColors(allowedColors);

        ac.review("Selected alliance: " + color.text);
        AllianceSaver.setAlliance(isBlue ? Alliance.BLUE : Alliance.RED);

        f.setStartingPose(StartPose());

        autoCommand = autoCommand(f, reader);
        cs.schedule(autoCommand.andThen(new InstantCommand(() -> autoFinished = true)));


        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(ClawConfiguration.VerticalRotation.UP);
        VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(ClawConfiguration.HorizontalRotation.NORMAL);
        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetState(ClawConfiguration.GripperState.CLOSED_LOOSE);

        waitForStart();
        autoTimer.reset();

        GlobalConfig.DEBUG_MODE = false;
        VLRSubsystem.getInstance(BlinkinSubsystem.class).setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);

        Init();
        while (opModeInInit()) {
            InitLoop();
            if (isStopRequested()) {ac.setStopRequested(true);}
        }

        waitForStart();
        boolean poseSaved = false;
        Start();
        while (opModeIsActive()) {
            Loop();
            f.update();

            if (autoFinished) {
                if (!prevAutoFinished) {
                    prevAutoFinished = true;
                    autoTime = autoTimer.seconds();
                }
                if (!poseSaved) {
                    PoseSaver.setPedroPose(f.getPose());
                    poseSaved = true;
                }
                System.out.println("TOTAL AUTO TIME: " + autoTime);
            }
        }
        Stop();
    }

    public void Loop() {
    }

    public void Init() {
    }

    public void Start() {
    }

    public void Stop() {
    }

    public void InitLoop() {
    }

    public abstract Command autoCommand(Follower f, LimelightYoloReader reader);

    public abstract Pose StartPose();

    public abstract boolean SpecimenOnly();
}
