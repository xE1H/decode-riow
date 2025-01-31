package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoOpModeRunner;
import org.firstinspires.ftc.teamcode.auto.commands.factory.ObservationCommandFactory;
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
        AutoOpModeRunner runner = new AutoOpModeRunner(new ObservationCommandFactory());
        runner.initialize(hardwareMap);
        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(ClawConfiguration.VerticalRotation.UP);
        waitForStart();
        runner.run(this::opModeIsActive, false);
    }

}
