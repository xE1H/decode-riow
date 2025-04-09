package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;

@Config
public class MainArmConfiguration {

    public static double EXCLUSION_ZONE_MIN_ANGLE = 0;
    public static double EXCLUSION_ZONE_MAX_ANGLE = 0;
    public static double EXCLUSION_ZONE_MIN_EXTENSION = 0;

    public static Vector2d ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER = new Vector2d(0, 0);
    public static Vector2d RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT = new Vector2d(0, 0);
    public static double ROBOT_LENGTH_CM = 0;

    public static double interpolationTimeConstant = 0; //time to move a distance of 1 in a unit circle


    public enum COORDINATE_IDENTIFIER {
        POLAR (0),
        CARTESIAN (1);

        public final int identifier;
        COORDINATE_IDENTIFIER(int extension) {
            this.identifier = extension;
        }
    }


    public enum COORDINATE_TYPE {
        ROBOT,
        REAL_WORLD
    }


    public enum OFFSET_REFERENCE_PLANE {
        FRONT (1),
        BACK (-1);

        public final int xScalar;
        OFFSET_REFERENCE_PLANE(int xScalar) {
            this.xScalar = xScalar;
        }
    }


    public enum TARGET_POSITION{
        INTAKE,
        RETRACT,
        SCORE_SAMPLE,
        SCORE_SPECIMEN
    }


    public enum OPERATION_MODE {
        NORMAL,
        HANG,
        HOLD_POINT
    }
}
