package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.AutoOpModeRunner;
import org.firstinspires.ftc.teamcode.auto.commands.factory.ObservationCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

@Config
@Photon
@Autonomous(name = "ObservationAuto", group = "!AUTO")
public class ObservationAuto extends VLRLinearOpMode {
    @Override
    public void run() {
//        ElapsedTime opModeTime = new ElapsedTime();
//        Alliance alliance;
//
//        AutoConfigurator configurator = new AutoConfigurator(telemetry, gamepad1);
//
//        AutoConfigurator.Choice choice = configurator.multipleChoice("Select alliance:",
//                new AutoConfigurator.Choice("Red"),
//                new AutoConfigurator.Choice("Blue"));
//        alliance = choice.text.equals("Red") ? Alliance.RED : Alliance.BLUE;
//
//        configurator.review("Alliance: " + alliance);

        AutoOpModeRunner runner = new AutoOpModeRunner(new ObservationCommandFactory());
        runner.initialize(hardwareMap);
        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(ClawConfiguration.VerticalRotation.UP);
        waitForStart();
        runner.run(this::opModeIsActive, false);
    }

}
