package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.DriveUtils;
import org.firstinspires.ftc.teamcode.utils.LauncherUtils;

/**
 * Autonomous OpMode that backs up and then shoots a full round of balls.
 * 
 * <p>Sequence:
 * <ol>
 *   <li>Back up for a set duration</li>
 *   <li>Spin up the launcher to target velocity</li>
 *   <li>Shoot multiple balls (full round) with proper timing</li>
 * </ol>
 * 
 * <p>This OpMode uses a modular structure with separate hardware and utility classes
 * for better organization and maintainability. Includes comprehensive error handling
 * and telemetry feedback.
 * 
 * @author Team 28448
 * @version 2.0
 */
@Autonomous(name = "Auto", group = "StarterBot")
public class Auto extends LinearOpMode {
    
    private RobotHardware hardware;
    private double lastTelemetryUpdate = 0;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        hardware = new RobotHardware(hardwareMap);
        
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        
        if (!hardware.init()) {
            telemetry.addData("ERROR", "Hardware initialization failed!");
            telemetry.addData("Status", "Check hardware connections");
            telemetry.update();
            
            // Wait for user to see error message
            sleep(5000);
            return;
        }
        
        // Verify hardware is ready
        if (!hardware.isDrivetrainReady()) {
            telemetry.addData("WARNING", "Drivetrain not fully initialized");
        }
        if (!hardware.isLauncherReady()) {
            telemetry.addData("WARNING", "Launcher system not fully initialized");
        }
        
        telemetry.addData("Status", "Initialized - Ready to start!");
        telemetry.addData("Balls to shoot", Constants.NUM_BALLS_FULL_ROUND);
        telemetry.addData("Backup time", "%.1f seconds", Constants.BACKUP_TIME_SECONDS);
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            try {
                // Step 1: Back up
                updateTelemetry("Backing up...");
                backUp();
                
                // Small delay after backing up
                sleep(Constants.POST_BACKUP_DELAY_MS);
                
                // Step 2: Spin up launcher
                updateTelemetry("Spinning up launcher...");
                if (!spinUpLauncher()) {
                    telemetry.addData("ERROR", "Launcher spin-up failed!");
                    telemetry.update();
                    return;
                }
                
                // Step 3: Shoot full round of balls
                updateTelemetry("Shooting balls...");
                shootFullRound();
                
                // Stop everything
                hardware.stopAll();
                
                updateTelemetry("Autonomous complete!");
                
            } catch (Exception e) {
                // Emergency stop on any error
                hardware.stopAll();
                telemetry.addData("ERROR", "Exception occurred: " + e.getMessage());
                telemetry.update();
            }
        }
    }
    
    /**
     * Backs up the robot for a set duration using Constants.
     * Includes telemetry updates during movement.
     */
    private void backUp() {
        double startTime = getRuntime();
        double elapsed;
        
        while (opModeIsActive() && 
               (elapsed = getRuntime() - startTime) < Constants.BACKUP_TIME_SECONDS) {
            
            DriveUtils.driveBackward(hardware, Math.abs(Constants.BACKUP_POWER));
            
            // Update telemetry periodically
            if (shouldUpdateTelemetry()) {
                telemetry.addData("Status", "Backing up...");
                telemetry.addData("Elapsed", "%.2f / %.2f sec", elapsed, Constants.BACKUP_TIME_SECONDS);
                telemetry.addData("Progress", "%.0f%%", (elapsed / Constants.BACKUP_TIME_SECONDS) * 100);
                telemetry.update();
            }
        }
        
        DriveUtils.stop(hardware);
    }
    
    /**
     * Spins up the launcher to target velocity and waits for it to be ready.
     * 
     * @return true if launcher reached ready state, false if timeout
     */
    private boolean spinUpLauncher() {
        if (!LauncherUtils.spinUp(hardware)) {
            return false;
        }
        
        // Wait for launcher to reach minimum velocity
        double startTime = getRuntime();
        double currentTime;
        boolean ready = false;
        
        while (opModeIsActive() && !ready) {
            currentTime = getRuntime();
            ready = LauncherUtils.isReady(hardware);
            
            // Check for timeout
            if (LauncherUtils.isSpinUpTimeout(startTime, currentTime, Constants.LAUNCHER_SPINUP_TIMEOUT)) {
                telemetry.addData("WARNING", "Launcher spin-up timeout!");
                telemetry.update();
                return false;
            }
            
            // Update telemetry periodically
            if (shouldUpdateTelemetry()) {
                double velocity = LauncherUtils.getVelocity(hardware);
                double percent = LauncherUtils.getVelocityPercent(hardware) * 100;
                
                telemetry.addData("Status", "Spinning up launcher...");
                telemetry.addData("Velocity", "%.1f / %.1f ticks/sec", velocity, Constants.LAUNCHER_MIN_VELOCITY);
                telemetry.addData("Progress", "%.0f%%", percent);
                telemetry.addData("Ready", ready ? "YES" : "NO");
                telemetry.update();
            }
            
            sleep(Constants.LAUNCHER_POLL_INTERVAL_MS);
        }
        
        if (ready) {
            telemetry.addData("Launcher", "Ready at %.1f ticks/sec", LauncherUtils.getVelocity(hardware));
            telemetry.update();
            return true;
        }
        
        return false;
    }
    
    /**
     * Shoots a full round of balls using Constants configuration.
     * Includes proper timing and launcher speed verification.
     */
    private void shootFullRound() {
        // Ensure launcher is spinning
        if (!LauncherUtils.isReady(hardware)) {
            LauncherUtils.spinUp(hardware);
            sleep(500); // Brief wait for spin-up
        }
        
        // Shoot each ball
        for (int i = 0; i < Constants.NUM_BALLS_FULL_ROUND && opModeIsActive(); i++) {
            // Update telemetry
            if (shouldUpdateTelemetry()) {
                telemetry.addData("Status", "Shooting ball %d of %d", i + 1, Constants.NUM_BALLS_FULL_ROUND);
                telemetry.addData("Launcher Velocity", "%.1f ticks/sec", LauncherUtils.getVelocity(hardware));
                telemetry.update();
            }
            
            // Ensure launcher is at speed before each shot
            if (!LauncherUtils.isReady(hardware)) {
                LauncherUtils.spinUp(hardware);
                sleep(500); // Wait for spin-up
            }
            
            // Feed ball
            LauncherUtils.startFeeding(hardware);
            sleep(LauncherUtils.getFeedTimeMs());
            LauncherUtils.stopFeeding(hardware);
            
            // Wait between shots (except after last shot)
            if (i < Constants.NUM_BALLS_FULL_ROUND - 1) {
                sleep(Constants.SHOT_DELAY_MS);
            }
        }
        
        telemetry.addData("Status", "Completed %d balls", Constants.NUM_BALLS_FULL_ROUND);
        telemetry.update();
    }
    
    /**
     * Update telemetry with status message.
     * 
     * @param message Status message to display
     */
    private void updateTelemetry(String message) {
        telemetry.addData("Status", message);
        telemetry.update();
        lastTelemetryUpdate = getRuntime();
    }
    
    /**
     * Check if telemetry should be updated (throttles updates).
     * 
     * @return true if enough time has passed since last update
     */
    private boolean shouldUpdateTelemetry() {
        double currentTime = getRuntime();
        if (currentTime - lastTelemetryUpdate >= Constants.TELEMETRY_UPDATE_INTERVAL_MS / 1000.0) {
            lastTelemetryUpdate = currentTime;
            return true;
        }
        return false;
    }
}
