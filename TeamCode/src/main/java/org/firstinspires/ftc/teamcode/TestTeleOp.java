/*
 * Testing TeleOp for Microbot
 * 
 * This OpMode allows individual testing of all motors and servos.
 * Use Controller 1 to test each component.
 * 
 * BUTTON MAPPING:
 * ===============
 * MOTORS (hold to run):
 *   D-Pad Up    -> Left Front Drive
 *   D-Pad Right -> Right Front Drive
 *   D-Pad Down  -> Left Back Drive
 *   D-Pad Left  -> Right Back Drive
 *   A Button    -> Launcher Motor
 *   
 * SERVOS (hold to run):
 *   X Button    -> Left Feeder Servo
 *   B Button    -> Right Feeder Servo
 *   
 * SPECIAL:
 *   Y Button    -> Test ALL drive motors together (mecanum forward)
 *   Left Bumper -> Toggle direction (FORWARD/REVERSE)
 *   Right Trigger -> Adjust motor power (0.0 to 1.0)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TestTeleOp", group = "StarterBot")
//@Disabled
public class TestTeleOp extends OpMode {

    // Motor power settings
    private static final double DEFAULT_MOTOR_POWER = 0.5;
    private static final double DEFAULT_SERVO_POWER = 0.5;

    // Hardware
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // State
    private boolean reverseDirection = false;
    private boolean lastBumperState = false;
    private String activeComponent = "None";

    @Override
    public void init() {
        // Initialize all hardware
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            telemetry.addData("left_front_drive", "OK");
        } catch (Exception e) {
            telemetry.addData("left_front_drive", "NOT FOUND");
        }

        try {
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            telemetry.addData("right_front_drive", "OK");
        } catch (Exception e) {
            telemetry.addData("right_front_drive", "NOT FOUND");
        }

        try {
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            telemetry.addData("left_back_drive", "OK");
        } catch (Exception e) {
            telemetry.addData("left_back_drive", "NOT FOUND");
        }

        try {
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            telemetry.addData("right_back_drive", "OK");
        } catch (Exception e) {
            telemetry.addData("right_back_drive", "NOT FOUND");
        }

        try {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            telemetry.addData("launcher", "OK");
        } catch (Exception e) {
            telemetry.addData("launcher", "NOT FOUND");
        }

        try {
            leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
            telemetry.addData("left_feeder", "OK");
        } catch (Exception e) {
            telemetry.addData("left_feeder", "NOT FOUND");
        }

        try {
            rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
            telemetry.addData("right_feeder", "OK");
        } catch (Exception e) {
            telemetry.addData("right_feeder", "NOT FOUND");
        }

        // Set zero power behavior for motors
        if (leftFrontDrive != null) leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (rightFrontDrive != null) rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (leftBackDrive != null) leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (rightBackDrive != null) rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (launcher != null) launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized - Ready for testing!");
        telemetry.addData("", "");
        telemetry.addData("Controls", "See button mapping above");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Toggle direction with left bumper (edge detection)
        boolean currentBumperState = gamepad1.left_bumper;
        if (currentBumperState && !lastBumperState) {
            reverseDirection = !reverseDirection;
        }
        lastBumperState = currentBumperState;

        // Calculate power based on right trigger (default to 0.5 if not pressed)
        double motorPower = gamepad1.right_trigger > 0.1 ? gamepad1.right_trigger : DEFAULT_MOTOR_POWER;
        double servoPower = gamepad1.right_trigger > 0.1 ? gamepad1.right_trigger : DEFAULT_SERVO_POWER;

        // Apply direction
        if (reverseDirection) {
            motorPower = -motorPower;
            servoPower = -servoPower;
        }

        // Reset active component
        activeComponent = "None";

        // Stop all motors/servos first
        stopAllMotors();
        stopAllServos();

        // Test individual motors with D-Pad
        if (gamepad1.dpad_up && leftFrontDrive != null) {
            leftFrontDrive.setPower(motorPower);
            activeComponent = "Left Front Drive";
        }
        else if (gamepad1.dpad_right && rightFrontDrive != null) {
            rightFrontDrive.setPower(motorPower);
            activeComponent = "Right Front Drive";
        }
        else if (gamepad1.dpad_down && leftBackDrive != null) {
            leftBackDrive.setPower(motorPower);
            activeComponent = "Left Back Drive";
        }
        else if (gamepad1.dpad_left && rightBackDrive != null) {
            rightBackDrive.setPower(motorPower);
            activeComponent = "Right Back Drive";
        }
        // Test launcher with A button
        else if (gamepad1.a && launcher != null) {
            launcher.setPower(motorPower);
            activeComponent = "Launcher Motor";
        }
        // Test left feeder with X button
        else if (gamepad1.x && leftFeeder != null) {
            leftFeeder.setPower(servoPower);
            activeComponent = "Left Feeder Servo";
        }
        // Test right feeder with B button
        else if (gamepad1.b && rightFeeder != null) {
            rightFeeder.setPower(servoPower);
            activeComponent = "Right Feeder Servo";
        }
        // Test all drive motors together with Y button
        else if (gamepad1.y) {
            if (leftFrontDrive != null) leftFrontDrive.setPower(motorPower);
            if (rightFrontDrive != null) rightFrontDrive.setPower(motorPower);
            if (leftBackDrive != null) leftBackDrive.setPower(motorPower);
            if (rightBackDrive != null) rightBackDrive.setPower(motorPower);
            activeComponent = "ALL Drive Motors";
        }

        // Telemetry display
        telemetry.addData("=== TEST TELEOP ===", "");
        telemetry.addData("Active Component", activeComponent);
        telemetry.addData("Direction", reverseDirection ? "REVERSE" : "FORWARD");
        telemetry.addData("Power Level", "%.2f", Math.abs(motorPower));
        telemetry.addData("", "");
        
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("D-Pad Up", "Left Front Drive");
        telemetry.addData("D-Pad Right", "Right Front Drive");
        telemetry.addData("D-Pad Down", "Left Back Drive");
        telemetry.addData("D-Pad Left", "Right Back Drive");
        telemetry.addData("A Button", "Launcher Motor");
        telemetry.addData("X Button", "Left Feeder Servo");
        telemetry.addData("B Button", "Right Feeder Servo");
        telemetry.addData("Y Button", "ALL Drive Motors");
        telemetry.addData("", "");
        telemetry.addData("Left Bumper", "Toggle Forward/Reverse");
        telemetry.addData("Right Trigger", "Adjust Power (0-100%)");
        telemetry.addData("", "");

        // Show encoder values if available
        telemetry.addData("=== ENCODER VALUES ===", "");
        if (leftFrontDrive != null) {
            telemetry.addData("Left Front Encoder", leftFrontDrive.getCurrentPosition());
        }
        if (rightFrontDrive != null) {
            telemetry.addData("Right Front Encoder", rightFrontDrive.getCurrentPosition());
        }
        if (leftBackDrive != null) {
            telemetry.addData("Left Back Encoder", leftBackDrive.getCurrentPosition());
        }
        if (rightBackDrive != null) {
            telemetry.addData("Right Back Encoder", rightBackDrive.getCurrentPosition());
        }
        if (launcher != null) {
            telemetry.addData("Launcher Encoder", launcher.getCurrentPosition());
            telemetry.addData("Launcher Velocity", "%.1f", launcher.getVelocity());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Just incase
        stopAllMotors();
        stopAllServos();
    }

    private void stopAllMotors() {
        if (leftFrontDrive != null) leftFrontDrive.setPower(0);
        if (rightFrontDrive != null) rightFrontDrive.setPower(0);
        if (leftBackDrive != null) leftBackDrive.setPower(0);
        if (rightBackDrive != null) rightBackDrive.setPower(0);
        if (launcher != null) launcher.setPower(0);
    }

    private void stopAllServos() {
        if (leftFeeder != null) leftFeeder.setPower(0);
        if (rightFeeder != null) rightFeeder.setPower(0);
    }
}

