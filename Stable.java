/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "Stable", group = "StarterBot")
//@Disabled
public class Stable extends OpMode {
    final double FEED_TIME_SECONDS = 0.18; //The feeder servos run this left_long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.

    final double FULL_SPEED = 0.75;
    double moveSpeed = 1.0;
    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or le ss power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1810;
    final double LAUNCHER_MIN_VELOCITY = 1800;
    final double LAUNCHER_TIMEOUT_SECONDS = 10.0; // Safety timeout - stops launcher if no launch after 10 seconds

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime launcherTimer = new ElapsedTime(); // Timer for launcher safety timeout
    ElapsedTime feederTimer = new ElapsedTime();
    
    private boolean launcherRunning = false; // Track if launcher is currently running
    
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    // Code to run REPEATEDLY after the driver hits START but before they hit STOP
    @Override
    public void loop() {

        moveSpeed = (1 - gamepad1.right_trigger/1.25);
        mecanumDrive(-gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */

        if (gamepad1.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
            launcherRunning = false;
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.x);

        /*
         * Safety timeout: Stop launcher if it's been running for 10 seconds without launching
         */
        if (launcherRunning && launcherTimer.seconds() > LAUNCHER_TIMEOUT_SECONDS) {
            launcher.setVelocity(STOP_SPEED);
            launcherRunning = false;
            launchState = LaunchState.IDLE;
            telemetry.addData("WARNING", "Launcher timed out after 10 seconds!");
        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        if (launcherRunning) {
            telemetry.addData("Launcher Timer", "%.1f / %.1f sec", launcherTimer.seconds(), LAUNCHER_TIMEOUT_SECONDS);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFrontPower = (-forward + strafe - rotate) / denominator;
        rightFrontPower = (-forward + strafe + rotate) / denominator;
        leftBackPower = (-forward - strafe - rotate) / denominator;
        rightBackPower = (-forward - strafe + rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower * moveSpeed);
        rightFrontDrive.setPower(rightFrontPower * moveSpeed);
        leftBackDrive.setPower(leftBackPower * moveSpeed);
        rightBackDrive.setPower(rightBackPower * moveSpeed);

    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    // Start launcher timer when beginning spin-up
                    if (!launcherRunning) {
                        launcherTimer.reset();
                        launcherRunning = true;
                    }
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // For negative velocities, check if velocity <= min (more negative = faster)
                if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                // Reset the timer since we're actually launching
                launcherTimer.reset();
                leftFeeder.setPower(0 - FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    // Launcher finished its job, reset tracking
                    launcherRunning = false;
                }
                break;
        }
    }
}
