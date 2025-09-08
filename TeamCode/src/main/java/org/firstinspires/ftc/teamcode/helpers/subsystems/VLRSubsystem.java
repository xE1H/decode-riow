package org.firstinspires.ftc.teamcode.helpers.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.logging.Logger;

/**
 * Abstract base class for subsystems.
 * Implements a singleton pattern for subsystem instances.
 *
 * @param <T> The specific subsystem type extending this class
 */
public abstract class VLRSubsystem<T extends VLRSubsystem<T>> extends SubsystemBase {
    /**
     * Map to store singleton instances of subsystems
     */

    protected Logger logger;
    private static final Map<Class<?>, VLRSubsystem<?>> instances = new HashMap<>();

    /**
     * Protected constructor to prevent direct instantiation.
     */
    protected VLRSubsystem() {
        // Protected constructor to prevent direct instantiation
    }

    /**
     * Gets the singleton instance of a specific subsystem.
     *
     * @param <E>   The type of the subsystem
     * @param clazz The class of the subsystem
     * @return The singleton instance of the specified subsystem
     * @throws RuntimeException if the instance cannot be retrieved
     */
    @SuppressWarnings("unchecked")
    public static <E extends VLRSubsystem<E>> E getInstance(Class<E> clazz) {
        try {
            return (E) instances.get(clazz);
        } catch (NullPointerException e) {
            throw new RuntimeException("Error getting instance of " + clazz.getName(), e);
        }
    }

    /**
     * Initializes the subsystem with the given hardware map.
     * This method should be implemented by subclasses to set up hardware components.
     *
     * @param hardwareMap The HardwareMap to use for initialization
     */
    protected abstract void initialize(HardwareMap hardwareMap);

    /**
     * Initializes all of the required subsystems with the given hardware map.
     * This method should be called once at the beginning of the OpMode to ensure all subsystems are ready.
     *
     * @param hardwareMap The HardwareMap to use for initialization
     */
    public static void initializeAll(HardwareMap hardwareMap) {
        for (VLRSubsystem<?> subsystem : instances.values()) {
            subsystem.initialize(hardwareMap);
        }
    }

    public static void initializeOne(HardwareMap hardwareMap, Class<? extends VLRSubsystem<?>> subsystem) {
        //noinspection unchecked
        requireSubsystems(subsystem);
        Objects.requireNonNull(instances.get(subsystem)).initialize(hardwareMap);
    }

    /**
     * Ensures that the specified subsystems are instantiated and available.
     * If a subsystem is not yet instantiated, it will be created.
     *
     * @param subsystems The classes of the subsystems to require
     * @throws RuntimeException if a subsystem instance cannot be created
     */
    @SuppressWarnings("unchecked")
    public static void requireSubsystems(Class<? extends VLRSubsystem<?>>... subsystems) {
        for (Class<? extends VLRSubsystem<?>> subsystem : subsystems) {
            if (!instances.containsKey(subsystem)) {
                try {
                    VLRSubsystem<?> instance = subsystem.getDeclaredConstructor().newInstance();
                    instance.logger = Logger.getLogger(subsystem.getSimpleName());
                    instances.put(subsystem, instance);
                } catch (Exception e) {
                    throw new RuntimeException("Error creating instance of " + subsystem.getName(), e);
                }
            }
        }
    }

    public static Logger getLogger(Class<? extends VLRSubsystem<?>> subsystem) {
        for (Class<?> sub : instances.keySet()) {
            if (sub == subsystem) {
                try {
                    return Objects.requireNonNull(instances.get(sub)).logger;
                } catch (Exception e) {
                    throw new RuntimeException("Error getting logger for " + subsystem.getName(), e);
                }
            }
        }
        throw new RuntimeException("Error getting logger for " + subsystem.getName() + ": Subsystem not found");
    }

    public static void clearSubsystems() {
        instances.clear();
    }
}
