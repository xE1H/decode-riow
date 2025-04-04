package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

@Config
public class SetArmPosition extends ConditionalCommand {
    public static double exclusionZoneMinAngle = 0;
    public static double exclusionZoneMaxAngle = 0;
    public static double exclusionZoneMinExtension = 0;

    public SetArmPosition(){
        super(
                new SequentialCommandGroup(

                ),
                new SequentialCommandGroup(

                ),
                ()-> isCameraInDanger()
        );
    }

    private static boolean isBetween(double num, double num1, double num2){
        return num > Math.min(num1, num2) && num < Math.max(num1, num2);
    }

    private static boolean isCameraInDanger(double target, double prevTarget, double extension){
        return (isBetween(exclusionZoneMinAngle, prevTarget, target) || isBetween(exclusionZoneMaxAngle, prevTarget, target)) || extension > exclusionZoneMinExtension;
    }
}
