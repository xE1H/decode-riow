package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Auto PID Tuner", group = "PIDF Tuning")
public class AutoPidTuner extends OpMode {
    private MultipleTelemetry telemetryA;
    private Follower follower;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Tuning parameters
    public enum TuningPhase {TRANSLATIONAL, HEADING, DRIVE, COMPLETE}

    public static TuningPhase currentPhase = TuningPhase.TRANSLATIONAL;
    private boolean forwardDirection = true;
    private double maxOvershoot = 0;
    private double settlingStartTime = 0;
    private boolean measurementInProgress = false;

    // Distance for straight-line movement tests during translational PID tuning
    // Robot will move back/forth this distance (in inches) to test path following
    public static double TRANSLATIONAL_TEST_DISTANCE = 48.0; // inches

    // Target angle for rotational tests during heading PID tuning
    // Robot will attempt to turn ± this angle (in radians) to test rotational control
    public static double HEADING_TEST_ANGLE = Math.toRadians(90);

    // Acceptable error threshold for considering the system "settled"
    // For translational: ±0.1 inches from target position
    public static double SETTLING_THRESHOLD_TRANSLATIONAL = 0.1; // inches

    // Acceptable error threshold for considering the system "settled"
    // For heading: ±0.1 radians (~5.7°) from target angle
    public static double SETTLING_THRESHOLD_HEADING = Math.PI / 90; // radians, equivalent to 2°


    // Time required maintaining position within SETTLING_THRESHOLD
    // to consider a movement complete (in milliseconds)
    public static int SETTLING_DURATION = 1000; // ms

    // Maximum allowed difference between actual vs target velocity
    // during drive PID tuning (in inches/second). If velocity remains
    // within ±0.5 in/s of target, consider velocity control adequate
    public static double VELOCITY_ERROR_THRESHOLD = 0.5; // inches/sec

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeTuningParameters();
    }

    private void initializeTuningParameters() {
        // Reset all PID gains to safe defaults
        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.1, 0, 0.01, 0);
        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(1.0, 0, 0.05, 0);
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.02, 0, 0.0001, 0.6, 0);

        // Configure PID usage flags
        Follower.useTranslational = true;
        Follower.useHeading = false;
        Follower.useDrive = false;
        Follower.useCentripetal = false;
    }

    @Override
    public void loop() {
        switch (currentPhase) {
            case TRANSLATIONAL:
                Follower.useTranslational = true;
                Follower.useHeading = false;
                Follower.useDrive = false;
                Follower.useCentripetal = false;
                tuneTranslationalControl();
                break;
            case HEADING:
                Follower.useTranslational = true;
                Follower.useHeading = true;
                Follower.useDrive = false;
                Follower.useCentripetal = false;
                tuneHeadingControl();
                break;
            case DRIVE:
                Follower.useTranslational = true;
                Follower.useHeading = true;
                Follower.useDrive = true;
                Follower.useCentripetal = false;
                tuneDriveControl();
                break;
            case COMPLETE:
                completeTuning();
                break;
        }

        follower.update();
        displayTelemetry();
    }

    private void tuneTranslationalControl() {
        if (!follower.isBusy()) {
            if (!measurementInProgress) {
                Path path = createStraightPath(
                        forwardDirection ? 0 : TRANSLATIONAL_TEST_DISTANCE,
                        forwardDirection ? TRANSLATIONAL_TEST_DISTANCE : 0
                );
                follower.followPath(path);
                forwardDirection = !forwardDirection;
                measurementInProgress = true;
                maxOvershoot = 0;
                stateTimer.reset();
            } else {
                double currentError = follower.getTranslationalError().getMagnitude();

                if (currentError > maxOvershoot) maxOvershoot = currentError;
                if (currentError < SETTLING_THRESHOLD_TRANSLATIONAL) {
                    handleSettling(() -> {
                        adjustTranslationalGains();
                        measurementInProgress = false;
                    });
                } else {
                    resetSettlingTimer();
                }
            }
        }
    }

    private void tuneHeadingControl() {
        if (!follower.isBusy()) {
            if (!measurementInProgress) {
                double targetHeading = forwardDirection ? HEADING_TEST_ANGLE : -HEADING_TEST_ANGLE;
                follower.holdPoint(new Pose(0, 0, targetHeading));
                forwardDirection = !forwardDirection;
                measurementInProgress = true;
                maxOvershoot = 0;
                stateTimer.reset();
            } else {
                double currentError = Math.abs(follower.headingError);

                if (currentError > maxOvershoot) maxOvershoot = currentError;
                if (currentError < SETTLING_THRESHOLD_HEADING) {
                    handleSettling(() -> {
                        adjustHeadingGains();
                        measurementInProgress = false;
                    });
                } else {
                    resetSettlingTimer();
                }
            }
        }
    }

    private void tuneDriveControl() {
        if (!follower.isBusy()) {
            if (!measurementInProgress) {
                Path path = createStraightPath(
                        forwardDirection ? 0 : TRANSLATIONAL_TEST_DISTANCE,
                        forwardDirection ? TRANSLATIONAL_TEST_DISTANCE : 0
                );
                follower.followPath(path);
                forwardDirection = !forwardDirection;
                measurementInProgress = true;
                maxOvershoot = 0;
                stateTimer.reset();
            } else {
                double currentError = Math.abs(follower.driveError);

                if (currentError > maxOvershoot) maxOvershoot = currentError;
                if (currentError < VELOCITY_ERROR_THRESHOLD) {
                    handleSettling(() -> {
                        adjustDriveGains();
                        measurementInProgress = false;
                    });
                } else {
                    resetSettlingTimer();
                }
            }
        }
    }

    private void handleSettling(Runnable completionAction) {
        if (settlingStartTime == 0) {
            settlingStartTime = stateTimer.milliseconds();
        }
        if (stateTimer.milliseconds() - settlingStartTime > SETTLING_DURATION) {
            completionAction.run();
            settlingStartTime = 0;
        }
    }

    private void resetSettlingTimer() {
        settlingStartTime = 0;
    }

    // Gain adjustment methods
    private void adjustTranslationalGains() {
        if (maxOvershoot > SETTLING_THRESHOLD_TRANSLATIONAL * 2) {
            FollowerConstants.translationalPIDFCoefficients.P *= 0.9;
            FollowerConstants.translationalPIDFCoefficients.D *= 1.1;
        } else {
            FollowerConstants.translationalPIDFCoefficients.P *= 1.1;
        }
        constrainTranslationalGains();
    }

    private void adjustHeadingGains() {
        if (maxOvershoot > SETTLING_THRESHOLD_HEADING * 2) {
            FollowerConstants.headingPIDFCoefficients.P *= 0.9;
            FollowerConstants.headingPIDFCoefficients.D *= 1.1;
        } else {
            FollowerConstants.headingPIDFCoefficients.P *= 1.1;
        }
        constrainHeadingGains();
    }

    private void adjustDriveGains() {
        if (maxOvershoot > VELOCITY_ERROR_THRESHOLD * 2) {
            FollowerConstants.drivePIDFCoefficients.P *= 0.9;
            FollowerConstants.drivePIDFCoefficients.D *= 1.1;
        } else {
            FollowerConstants.drivePIDFCoefficients.P *= 1.1;
        }
        constrainDriveGains();
    }

    // Safety constraints
    private void constrainTranslationalGains() {
        FollowerConstants.translationalPIDFCoefficients.P =
                Math.min(Math.max(FollowerConstants.translationalPIDFCoefficients.P, 0.1), 2.0);
        FollowerConstants.translationalPIDFCoefficients.D =
                Math.min(Math.max(FollowerConstants.translationalPIDFCoefficients.D, 0.0), 0.5);
    }

    private void constrainHeadingGains() {
        FollowerConstants.headingPIDFCoefficients.P =
                Math.min(Math.max(FollowerConstants.headingPIDFCoefficients.P, 0.1), 5.0);
        FollowerConstants.headingPIDFCoefficients.D =
                Math.min(Math.max(FollowerConstants.headingPIDFCoefficients.D, 0.0), 0.5);
    }

    private void constrainDriveGains() {
        FollowerConstants.drivePIDFCoefficients.P =
                Math.min(Math.max(FollowerConstants.drivePIDFCoefficients.P, 0.01), 0.1);
        FollowerConstants.drivePIDFCoefficients.D =
                Math.min(Math.max(FollowerConstants.drivePIDFCoefficients.D, 0.0), 0.01);
    }

    private Path createStraightPath(double start, double end) {
        return new Path(new BezierLine(
                new Point(start, 0, Point.CARTESIAN),
                new Point(end, 0, Point.CARTESIAN)
        ));
    }

    private void completeTuning() {
        follower.breakFollowing();
        displayFinalResults();
    }

    private void displayTelemetry() {
        telemetryA.addData("Current Phase", currentPhase);
        telemetryA.addData("Max Overshoot", maxOvershoot);
        telemetryA.addData("Settling Timer", stateTimer.milliseconds() - settlingStartTime);

        switch (currentPhase) {
            case TRANSLATIONAL:
                telemetryA.addLine("Translational PID:");
                telemetryA.addData("P", FollowerConstants.translationalPIDFCoefficients.P);
                telemetryA.addData("D", FollowerConstants.translationalPIDFCoefficients.D);
                break;
            case HEADING:
                telemetryA.addLine("Heading PID:");
                telemetryA.addData("P", FollowerConstants.headingPIDFCoefficients.P);
                telemetryA.addData("D", FollowerConstants.headingPIDFCoefficients.D);
                break;
            case DRIVE:
                telemetryA.addLine("Drive PID:");
                telemetryA.addData("P", FollowerConstants.drivePIDFCoefficients.P);
                telemetryA.addData("D", FollowerConstants.drivePIDFCoefficients.D);
                break;
        }

        telemetryA.update();
    }

    private void displayFinalResults() {
        telemetryA.addLine("TUNING COMPLETE");
        telemetryA.addLine("Final Parameters:");
        telemetryA.addData("Translational P", FollowerConstants.translationalPIDFCoefficients.P);
        telemetryA.addData("Translational D", FollowerConstants.translationalPIDFCoefficients.D);
        telemetryA.addData("Heading P", FollowerConstants.headingPIDFCoefficients.P);
        telemetryA.addData("Heading D", FollowerConstants.headingPIDFCoefficients.D);
        telemetryA.addData("Drive P", FollowerConstants.drivePIDFCoefficients.P);
        telemetryA.addData("Drive D", FollowerConstants.drivePIDFCoefficients.D);
        telemetryA.update();
    }

    private void transitionToNextPhase() {
        switch (currentPhase) {
            case TRANSLATIONAL:
                currentPhase = TuningPhase.HEADING;
                Follower.useTranslational = false;
                Follower.useHeading = true;
                break;
            case HEADING:
                currentPhase = TuningPhase.DRIVE;
                Follower.useHeading = false;
                Follower.useDrive = true;
                Follower.useTranslational = true; // Enable for drive tuning
                break;
            case DRIVE:
                currentPhase = TuningPhase.COMPLETE;
                break;
        }
        resetState();
    }

    private void resetState() {
        forwardDirection = true;
        measurementInProgress = false;
        maxOvershoot = 0;
        settlingStartTime = 0;
        stateTimer.reset();
    }
}