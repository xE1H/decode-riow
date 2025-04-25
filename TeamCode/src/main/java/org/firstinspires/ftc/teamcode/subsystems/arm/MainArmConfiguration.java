package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;

@Config
public class MainArmConfiguration {

    public static double EXCLUSION_ZONE_MIN_ANGLE = 20;
    public static double EXCLUSION_ZONE_MAX_ANGLE = 48;
    public static double EXCLUSION_ZONE_MIN_EXTENSION = 0.05;

    //dimensions taken from cad
    public static Vector2d ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER = new Vector2d(15.25, 13.76);
    public static Vector2d RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT = new Vector2d(34.844, -9);
    public static double ROBOT_LENGTH_CM = 44.9;

    public static double interpolationTimeConstant = 3; //time to move a distance of 1 in a unit circle


    public enum OFFSET_REFERENCE_PLANE {
        FRONT (1),
        BACK (-1);

        public final int xScalar;
        OFFSET_REFERENCE_PLANE(int xScalar) {
            this.xScalar = xScalar;
        }
    }

    public enum OPERATION_MODE {
        NORMAL,
        HANG,
        HOLD_POINT
    }

    public enum SAMPLE_SCORE_HEIGHT {
        LOW_BASKET (0.4),
        HIGH_BASKET (1);

        public final double extension;
        SAMPLE_SCORE_HEIGHT (double extension){
            this.extension = extension;
        }
    }


    public enum GAME_PIECE_TYPE {
        SAMPLE,
        SPECIMEN
    }
}
