package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.RED;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.YELLOW;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.persistence.AllianceSaver;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.Arrays;

//@Disabled
@TeleOp(group = "!TELEOP")
public class ManualAllianceInverter extends LinearOpMode {
    LimelightYoloReader reader;


    @Override
    public void runOpMode() {
        reader = new LimelightYoloReader();
        waitForStart();

        if (opModeIsActive()) {
            if (AllianceSaver.getAlliance() == Alliance.RED) {
                AllianceSaver.setAlliance(Alliance.BLUE);
                reader.setAllowedColors(Arrays.asList(BLUE, YELLOW));
                telemetry.addLine("SET ALLIANCE TO BLUE");
            } else if (AllianceSaver.getAlliance() == Alliance.BLUE) {
                AllianceSaver.setAlliance(Alliance.RED);
                reader.setAllowedColors(Arrays.asList(RED, YELLOW));
                telemetry.addLine("SET ALLIANCE TO RED");
            }

            telemetry.update();
        }
    }
}