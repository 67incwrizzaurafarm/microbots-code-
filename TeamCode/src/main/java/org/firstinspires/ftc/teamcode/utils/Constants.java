package org.firstinspires.ftc.teamcode.utils;

/**
 * Centralized constants for the robot.
 * All tunable values should be defined here for easy adjustment.
 * 
 * <p>Organized by subsystem for easy navigation:
 * <ul>
 *   <li>Launcher constants - velocities, feed times, speeds</li>
 *   <li>Drive constants - powers, times, encoder values</li>
 *   <li>Autonomous constants - sequences, delays, counts</li>
 *   <li>TeleOp constants - control mappings, multipliers</li>
 * </ul>
 * 
 * @author Team 28448
 * @version 2.0
 */
public class Constants {
    
    // ============================================================================
    // LAUNCHER CONSTANTS
    // ============================================================================
    
    /** Target velocity for launcher motor (encoder ticks per second, negative = reverse direction) */
    public static final double LAUNCHER_TARGET_VELOCITY = -1745;
    
    /** Minimum velocity threshold before launcher is considered ready to shoot (negative = reverse direction) */
    public static final double LAUNCHER_MIN_VELOCITY = -1675;
    
    /** Time in seconds that feeder servos run when feeding a ball */
    public static final double FEED_TIME_SECONDS = 0.2;
    
    /** Full speed power for feeder servos (0.0 to 1.0) */
    public static final double FEEDER_FULL_SPEED = 0.75;
    
    /** Stop speed (zero power) for feeder servos */
    public static final double FEEDER_STOP_SPEED = 0.0;
    
    /** Safety timeout in seconds - stops launcher if no launch after this time */
    public static final double LAUNCHER_TIMEOUT_SECONDS = 10.0;
    
    /** Maximum time to wait for launcher spin-up in seconds */
    public static final double LAUNCHER_SPINUP_TIMEOUT = 5.0;
    
    /** Polling interval for checking launcher velocity (milliseconds) */
    public static final long LAUNCHER_POLL_INTERVAL_MS = 50;
    
    // ============================================================================
    // DRIVE CONSTANTS
    // ============================================================================
    
    /** Default backup power (negative = reverse) */
    public static final double BACKUP_POWER = -0.5;
    
    /** Default backup time in seconds */
    public static final double BACKUP_TIME_SECONDS = 1.0;
    
    /** Default forward drive power */
    public static final double FORWARD_POWER = 0.5;
    
    /** Maximum drive power (safety limit) */
    public static final double MAX_DRIVE_POWER = 1.0;
    
    /** Minimum drive power threshold (below this, motors won't move) */
    public static final double MIN_DRIVE_POWER = 0.05;
    
    // ============================================================================
    // AUTONOMOUS CONSTANTS
    // ============================================================================
    
    /** Number of balls to shoot in a full round */
    public static final int NUM_BALLS_FULL_ROUND = 5;
    
    /** Delay between shots in milliseconds */
    public static final long SHOT_DELAY_MS = 500;
    
    /** Delay after backing up before starting launcher (milliseconds) */
    public static final long POST_BACKUP_DELAY_MS = 200;
    
    /** Telemetry update interval in milliseconds */
    public static final long TELEMETRY_UPDATE_INTERVAL_MS = 100;
    
    // ============================================================================
    // TELEOP CONSTANTS
    // ============================================================================
    
    /** Speed multiplier calculation base (for right trigger speed control) */
    public static final double SPEED_MULTIPLIER_BASE = 1.25;
    
    /** Minimum speed multiplier (when trigger is fully pressed) */
    public static final double MIN_SPEED_MULTIPLIER = 0.2;
    
    // ============================================================================
    // UTILITY METHODS
    // ============================================================================
    
    /**
     * Clamp a value between min and max.
     * 
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Clamp drive power to safe range.
     * 
     * @param power Power to clamp
     * @return Clamped power value
     */
    public static double clampDrivePower(double power) {
        return clamp(power, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
    }
    
    // ============================================================================
    // PRIVATE CONSTRUCTOR
    // ============================================================================
    
    /**
     * Prevent instantiation - this is a utility class with only static members.
     */
    private Constants() {
        throw new UnsupportedOperationException("Constants class cannot be instantiated");
    }
}
