package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

/**
 * Utility class for launcher operations.
 * Provides methods for controlling the launcher and shooting balls with proper error handling.
 * 
 * <p>Features:
 * <ul>
 *   <li>Velocity-based launcher control</li>
 *   <li>Ready state checking</li>
 *   <li>Ball feeding with timing</li>
 *   <li>Safety timeouts</li>
 * </ul>
 * 
 * @author Team 28448
 * @version 2.0
 */
public class LauncherUtils {
    
    /**
     * Spin up the launcher to target velocity.
     * 
     * @param hardware RobotHardware instance (must not be null)
     * @return true if launcher was successfully commanded, false if hardware unavailable
     */
    public static boolean spinUp(RobotHardware hardware) {
        if (hardware == null || hardware.launcher == null) {
            return false;
        }
        
        try {
            hardware.launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            return true;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Check if launcher has reached minimum velocity threshold.
     * For negative velocities, checks if velocity is at or below (more negative than) minimum.
     * 
     * @param hardware RobotHardware instance
     * @return true if launcher is at or above minimum velocity (for negative: at or below), false otherwise
     */
    public static boolean isReady(RobotHardware hardware) {
        if (hardware == null || hardware.launcher == null) {
            return false;
        }
        
        try {
            double velocity = hardware.launcher.getVelocity();
            // For negative velocities, we check if velocity <= min (more negative = faster)
            // For positive velocities, we check if velocity >= min
            if (LAUNCHER_TARGET_VELOCITY < 0) {
                return velocity <= LAUNCHER_MIN_VELOCITY;
            } else {
                return velocity >= LAUNCHER_MIN_VELOCITY;
            }
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Get current launcher velocity.
     * 
     * @param hardware RobotHardware instance
     * @return Current velocity in encoder ticks per second, or 0.0 if unavailable
     */
    public static double getVelocity(RobotHardware hardware) {
        if (hardware == null || hardware.launcher == null) {
            return 0.0;
        }
        
        try {
            return hardware.launcher.getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }
    
    /**
     * Get velocity as a percentage of target velocity.
     * Handles both positive and negative velocities correctly.
     * 
     * @param hardware RobotHardware instance
     * @return Percentage (0.0 to 1.0+) of target velocity (always positive)
     */
    public static double getVelocityPercent(RobotHardware hardware) {
        double velocity = getVelocity(hardware);
        // Use absolute values to get percentage, then preserve sign if needed
        return Math.abs(velocity) / Math.abs(LAUNCHER_TARGET_VELOCITY);
    }
    
    /**
     * Check if launcher has been spinning up for the given duration.
     * Note: This method does not sleep - caller should handle timing.
     * 
     * @param startTime Start time in seconds (use getRuntime() from OpMode)
     * @param currentTime Current time in seconds (use getRuntime() from OpMode)
     * @param timeoutSeconds Maximum time to wait in seconds
     * @return true if timeout has been reached
     */
    public static boolean isSpinUpTimeout(double startTime, double currentTime, double timeoutSeconds) {
        return (currentTime - startTime) >= timeoutSeconds;
    }
    
    /**
     * Start feeding a ball (caller should wait and then call stopFeeding).
     * 
     * @param hardware RobotHardware instance
     * @return true if feeding started successfully
     */
    public static boolean startFeeding(RobotHardware hardware) {
        if (hardware == null || !hardware.isLauncherReady()) {
            return false;
        }
        
        try {
            hardware.leftFeeder.setPower(0 - FEEDER_FULL_SPEED);
            hardware.rightFeeder.setPower(FEEDER_FULL_SPEED);
            return true;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Stop feeding a ball.
     * 
     * @param hardware RobotHardware instance
     * @return true if feeding stopped successfully
     */
    public static boolean stopFeeding(RobotHardware hardware) {
        if (hardware == null || !hardware.isLauncherReady()) {
            return false;
        }
        
        try {
            hardware.leftFeeder.setPower(FEEDER_STOP_SPEED);
            hardware.rightFeeder.setPower(FEEDER_STOP_SPEED);
            return true;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Get the feed time in seconds.
     * 
     * @return Feed time in seconds
     */
    public static double getFeedTimeSeconds() {
        return FEED_TIME_SECONDS;
    }
    
    /**
     * Get the feed time in milliseconds.
     * 
     * @return Feed time in milliseconds
     */
    public static long getFeedTimeMs() {
        return (long)(FEED_TIME_SECONDS * 1000);
    }
    
    /**
     * Stop the launcher safely.
     * 
     * @param hardware RobotHardware instance
     * @return true if launcher stopped successfully
     */
    public static boolean stop(RobotHardware hardware) {
        if (hardware == null) {
            return false;
        }
        
        try {
            hardware.stopLauncher();
            return true;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Check if launcher system is ready (hardware initialized and available).
     * 
     * @param hardware RobotHardware instance
     * @return true if launcher system is ready
     */
    public static boolean isSystemReady(RobotHardware hardware) {
        return hardware != null && hardware.isLauncherReady();
    }
    
    // ========== PRIVATE CONSTRUCTOR ==========
    
    /**
     * Prevent instantiation - this is a utility class.
     */
    private LauncherUtils() {
        throw new UnsupportedOperationException("LauncherUtils class cannot be instantiated");
    }
}
