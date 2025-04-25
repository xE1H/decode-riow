package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Points_specimen.START_POSE;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.specimen.AutonomousPeriodActionSpecimen;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

import java.util.function.Consumer;

import pedroPathing.tuners.constants.FConstants;
import pedroPathing.tuners.constants.LConstants;


@TeleOp(name = "VLR_SpecimenAuto", group = "!TELEOP")
@Photon
public class VLRSpecimenAuto extends VLRLinearOpMode {
    CommandScheduler cs;
    Follower f;
    ElapsedTime autoTimer = new ElapsedTime();

    Command autoCommand;
    boolean autoFinished = false;
    boolean prevAutoFinished = false;
    double autoTime = 0;

//    public static double velocityThreshold = 9999999;
//    public static double headingThreshold = 9999999;
//    public static double driveThreshold = 9999999;
//    public static double translationalThreshold = 99999999;

    /**
     * @noinspection unchecked
     */
    @Override
    public void run() {
        Constants.setConstants(FConstants.class, LConstants.class);

        cs = CommandScheduler.getInstance();

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setStartingPose(START_POSE);

        autoCommand = new AutonomousPeriodActionSpecimen(f);
        //autoCommand = new SetArmPosition().scoreSpecimenFront();
        cs.schedule(autoCommand.andThen(new InstantCommand(()-> autoFinished = true)));

        waitForStart();
        autoTimer.reset();


        while (opModeIsActive()) {
            f.update();

            if (autoFinished){
                if (!prevAutoFinished){
                    prevAutoFinished = true;
                    autoTime = autoTimer.seconds();
                }
                System.out.println("TOTAL AUTO TIME: " + autoTime);
            }
        }
    }
}
