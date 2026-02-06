package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

/**
 * Utility class for drive-related operations.
 * Provides methods for controlling the robot's drivetrain with safety checks.
 * 
 * <p>Features:
 * <ul>
 *   <li>Mecanum drive kinematics</li>
 *   <li>Simple directional driving</li>
 *   <li>Power clamping for safety</li>
 *   <li>Null checks for hardware</li>
 * </ul>
 * 
 * @author Team 28448
 * @version 2.0
 */
public class DriveUtils {
    
    /**
     * Drive the robot using mecanum drive kinematics.
     * Powers are automatically normalized to prevent exceeding motor limits.
     * 
     * @param hardware RobotHardware instance (must not be null)
     * @param forward Forward/backward power (-1.0 to 1.0, positive = forward)
     * @param strafe Left/right strafe power (-1.0 to 1.0, positive = right)
     * @param rotate Rotation power (-1.0 to 1.0, positive = clockwise)
     * @param speedMultiplier Speed multiplier (0.0 to 1.0) for overall speed control
     * @throws IllegalArgumentException if hardware is null
     */
    public static void mecanumDrive(RobotHardware hardware, double forward, double strafe, double rotate, double speedMultiplier) {
        if (hardware == null) {
            throw new IllegalArgumentException("RobotHardware cannot be null");
        }
        
        if (!hardware.isDrivetrainReady()) {
            return; // Silently fail if drivetrain not ready
        }
        
        // Clamp inputs to valid range
        forward = clampDrivePower(forward);
        strafe = clampDrivePower(strafe);
        rotate = clampDrivePower(rotate);
        speedMultiplier = clamp(speedMultiplier, 0.0, 1.0);
        
        // Normalize motor powers to prevent values exceeding 1.0
        // This ensures all motors maintain the same ratio even at max power
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
        
        // Calculate individual motor powers using mecanum kinematics
        double leftFrontPower = (-forward + strafe - rotate) / denominator;
        double rightFrontPower = (-forward + strafe + rotate) / denominator;
        double leftBackPower = (-forward - strafe - rotate) / denominator;
        double rightBackPower = (-forward - strafe + rotate) / denominator;
        
        // Apply speed multiplier and set motor powers
        hardware.leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
        hardware.rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
        hardware.leftBackDrive.setPower(leftBackPower * speedMultiplier);
        hardware.rightBackDrive.setPower(rightBackPower * speedMultiplier);
    }
    
    /**
     * Drive the robot using mecanum drive kinematics at full speed.
     * 
     * @param hardware RobotHardware instance
     * @param forward Forward/backward power
     * @param strafe Left/right strafe power
     * @param rotate Rotation power
     */
    public static void mecanumDrive(RobotHardware hardware, double forward, double strafe, double rotate) {
        mecanumDrive(hardware, forward, strafe, rotate, 1.0);
    }
    
    /**
     * Drive the robot in a simple forward/backward direction.
     * 
     * @param hardware RobotHardware instance
     * @param power Power level (-1.0 to 1.0, positive = forward)
     */
    public static void driveStraight(RobotHardware hardware, double power) {
        if (hardware == null || !hardware.isDrivetrainReady()) {
            return;
        }
        
        power = clampDrivePower(power);
        
        hardware.leftFrontDrive.setPower(power);
        hardware.rightFrontDrive.setPower(power);
        hardware.leftBackDrive.setPower(power);
        hardware.rightBackDrive.setPower(power);
    }
    
    /**
     * Drive the robot backward (reverse).
     * 
     * @param hardware RobotHardware instance
     * @param power Power level (0.0 to 1.0, will be made negative)
     */
    public static void driveBackward(RobotHardware hardware, double power) {
        driveStraight(hardware, -Math.abs(power));
    }
    
    /**
     * Drive the robot forward.
     * 
     * @param hardware RobotHardware instance
     * @param power Power level (0.0 to 1.0)
     */
    public static void driveForward(RobotHardware hardware, double power) {
        driveStraight(hardware, Math.abs(power));
    }
    
    /**
     * Rotate the robot in place (no translation).
     * 
     * @param hardware RobotHardware instance
     * @param power Rotation power (-1.0 to 1.0, positive = clockwise)
     */
    public static void rotateInPlace(RobotHardware hardware, double power) {
        mecanumDrive(hardware, 0, 0, power);
    }
    
    /**
     * Strafe left or right without rotation or forward/backward movement.
     * 
     * @param hardware RobotHardware instance
     * @param power Strafe power (-1.0 to 1.0, positive = right)
     */
    public static void strafe(RobotHardware hardware, double power) {
        mecanumDrive(hardware, 0, power, 0);
    }
    
    /**
     * Stop all drivetrain motors safely.
     * 
     * @param hardware RobotHardware instance
     */
    public static void stop(RobotHardware hardware) {
        if (hardware != null) {
            hardware.stopDrivetrain();
        }
    }
    
    /**
     * Get average encoder position from all drive motors.
     * Useful for odometry or distance tracking.
     * 
     * @param hardware RobotHardware instance
     * @return Average encoder position, or 0 if encoders unavailable
     */
    public static int getAverageEncoderPosition(RobotHardware hardware) {
        if (hardware == null || !hardware.isDrivetrainReady()) {
            return 0;
        }
        
        int sum = hardware.leftFrontDrive.getCurrentPosition() +
                  hardware.rightFrontDrive.getCurrentPosition() +
                  hardware.leftBackDrive.getCurrentPosition() +
                  hardware.rightBackDrive.getCurrentPosition();
        
        return sum / 4;
    }
    
    // ========== PRIVATE CONSTRUCTOR ==========
    
    /**
     * Prevent instantiation - this is a utility class.
     */
    private DriveUtils() {
        throw new UnsupportedOperationException("DriveUtils class cannot be instantiated");
    }
}
