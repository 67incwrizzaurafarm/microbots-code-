package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/**
 * Centralized hardware initialization and management class.
 * This class handles all hardware mapping and configuration with proper error handling.
 * 
 * @author Team 28448
 * @version 2.0
 */
public class RobotHardware {
    // ========== DRIVETRAIN MOTORS ==========
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;
    
    // ========== LAUNCHER SYSTEM ==========
    public DcMotorEx launcher;
    public CRServo leftFeeder;
    public CRServo rightFeeder;
    
    // ========== HARDWARE NAMES ==========
    // These must match the Robot Controller configuration
    private static final String LEFT_FRONT_DRIVE = "frontLeft";
    private static final String RIGHT_FRONT_DRIVE = "frontRight";
    private static final String LEFT_BACK_DRIVE = "backLeft";
    private static final String RIGHT_BACK_DRIVE = "backRight";
    private static final String LAUNCHER = "launcher";
    private static final String LEFT_FEEDER = "leftFeeder";
    private static final String RIGHT_FEEDER = "rightFeeder";
    
    // ========== CONFIGURATION ==========
    /** PIDF coefficients for launcher velocity control */
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(300, 0, 0, 10);
    
    private HardwareMap hardwareMap;
    private boolean initialized = false;
    
    /**
     * Constructor - requires HardwareMap from OpMode
     * 
     * @param hardwareMap HardwareMap from the OpMode
     */
    public RobotHardware(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        this.hardwareMap = hardwareMap;
    }
    
    /**
     * Initialize all hardware components.
     * Call this method once during OpMode initialization.
     * 
     * @return true if all hardware initialized successfully, false otherwise
     */
    public boolean init() {
        try {
            initDrivetrain();
            initLauncher();
            initFeeders();
            initialized = true;
            return true;
        } catch (Exception e) {
            initialized = false;
            return false;
        }
    }
    
    /**
     * Check if hardware has been initialized.
     * 
     * @return true if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }
    
    /**
     * Initialize drivetrain motors with proper error handling.
     */
    private void initDrivetrain() {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, LEFT_FRONT_DRIVE);
            rightFrontDrive = hardwareMap.get(DcMotor.class, RIGHT_FRONT_DRIVE);
            leftBackDrive = hardwareMap.get(DcMotor.class, LEFT_BACK_DRIVE);
            rightBackDrive = hardwareMap.get(DcMotor.class, RIGHT_BACK_DRIVE);
            
            // Set motor directions (matching Stable configuration)
            // Adjust these if your robot drives in the wrong direction
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            
            // Set zero power behavior to BRAKE for better control
            leftFrontDrive.setZeroPowerBehavior(BRAKE);
            rightFrontDrive.setZeroPowerBehavior(BRAKE);
            leftBackDrive.setZeroPowerBehavior(BRAKE);
            rightBackDrive.setZeroPowerBehavior(BRAKE);
            
            // Reset encoders (optional - uncomment if using encoders)
            // resetDrivetrainEncoders();
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize drivetrain: " + e.getMessage(), e);
        }
    }
    
    /**
     * Initialize launcher motor with velocity control.
     */
    private void initLauncher() {
        try {
            launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER);
            
            // Configure for velocity control
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setZeroPowerBehavior(BRAKE);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize launcher: " + e.getMessage(), e);
        }
    }
    
    /**
     * Initialize feeder servos.
     */
    private void initFeeders() {
        try {
            leftFeeder = hardwareMap.get(CRServo.class, LEFT_FEEDER);
            rightFeeder = hardwareMap.get(CRServo.class, RIGHT_FEEDER);
            
            // Set initial power to stop
            leftFeeder.setPower(0.0);
            rightFeeder.setPower(0.0);
            
            // Reverse left feeder so both work together to feed balls
            leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize feeders: " + e.getMessage(), e);
        }
    }
    
    /**
     * Reset all drivetrain encoders.
     * Call this before using encoder-based movement.
     */
    public void resetDrivetrainEncoders() {
        if (leftFrontDrive != null) {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightFrontDrive != null) {
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (leftBackDrive != null) {
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightBackDrive != null) {
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    
    /**
     * Check if all drivetrain motors are available.
     * 
     * @return true if all motors are initialized
     */
    public boolean isDrivetrainReady() {
        return leftFrontDrive != null && rightFrontDrive != null &&
               leftBackDrive != null && rightBackDrive != null;
    }
    
    /**
     * Check if launcher system is available.
     * 
     * @return true if launcher and feeders are initialized
     */
    public boolean isLauncherReady() {
        return launcher != null && leftFeeder != null && rightFeeder != null;
    }
    
    /**
     * Stop all drivetrain motors safely.
     */
    public void stopDrivetrain() {
        if (leftFrontDrive != null) leftFrontDrive.setPower(0);
        if (rightFrontDrive != null) rightFrontDrive.setPower(0);
        if (leftBackDrive != null) leftBackDrive.setPower(0);
        if (rightBackDrive != null) rightBackDrive.setPower(0);
    }
    
    /**
     * Stop launcher safely.
     */
    public void stopLauncher() {
        if (launcher != null) {
            launcher.setVelocity(0);
        }
    }
    
    /**
     * Stop feeder servos safely.
     */
    public void stopFeeders() {
        if (leftFeeder != null) leftFeeder.setPower(0.0);
        if (rightFeeder != null) rightFeeder.setPower(0.0);
    }
    
    /**
     * Emergency stop - stops all motors and servos immediately.
     * Use this in case of emergency or at the end of OpMode.
     */
    public void stopAll() {
        stopDrivetrain();
        stopLauncher();
        stopFeeders();
    }
}
