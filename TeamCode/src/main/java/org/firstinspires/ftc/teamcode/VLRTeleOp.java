package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.controls.PrimaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.controls.SecondaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ResetSlide;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;


/**
 * @noinspection unchecked
 */
@Photon
@TeleOp(name = "VLRTeleOp", group = "!TELEOP")
public class VLRTeleOp extends VLRLinearOpMode {
    // Controls
    PrimaryDriverTeleOpControls primaryDriver;
    SecondaryDriverTeleOpControls secondaryDriver;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class, ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, HangSubsystem.class  );
        VLRSubsystem.initializeAll(hardwareMap);

//        VLRSubsystem.getInstance(Chassis.class).enableFieldCentric();
        ArmSlideSubsystem ass = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
        primaryDriver = new PrimaryDriverTeleOpControls(gamepad1);
        secondaryDriver = new SecondaryDriverTeleOpControls(gamepad2);
        
        ass.setMotorPower(-0.6);
        ElapsedTime timeout = new ElapsedTime();
        while (!ass.getLimitSwitchState()) {
            sleep(10);
            if (timeout.milliseconds() > 1000) {
                break;
            }
        }
        ass.setMotorPower(0);
        ass.checkLimitSwitch();

        waitForStart();
        // since judges are pizdabolai

        while (opModeIsActive()) {
            primaryDriver.update();
            secondaryDriver.update();

            if (GlobalConfig.DEBUG_MODE) {
                telemetry.addData("current state", ArmState.get());
//                telemetry.addData("Slide pos", ass.getPosition());
            }
            telemetry.update();
        }
    }
}
