// FourWheelDrive.java
// Handles all mecanum wheel drive logic for the robot.
// Supports standard joystick driving, direction-based driving,
// AprilTag-assisted target locking, and a low-sensitivity mode for precision control.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class FourWheelDrive {

    // The four drive motors — one per corner of the mecanum drivetrain
    private DcMotorEx wheel_FrontRight;
    private DcMotorEx wheel_FrontLeft;
    private DcMotorEx wheel_BackRight;
    private DcMotorEx wheel_BackLeft;

    // Cached power values for each wheel, used for telemetry and calculations
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    // Low sensitivity mode: reduces wheel power for precise movements
    private boolean lowSensOn;
    private double lowSensMult;         // Dynamic multiplier (0–1), adjustable during a match
    private double lowSensPermaMult;    // Fixed reduced-power multiplier when low sensitivity is toggled on

    // Controls forward/backward orientation: 1 = normal, -1 = flipped (robot drives "backwards")
    private int direction = 1;

    // Tracks which direction the robot was last turning (used in lockDrive for tag tracking)
    private int lastTurnDirection = 0;

    /**
     * Constructor: initializes all four drive motors with correct directions
     * for a mecanum drivetrain. Motors are NOT running with encoders by default
     * (encoder mode lines are commented out).
     *
     * @param wheel_FrontRight  Front-right DcMotorEx
     * @param wheel_FrontLeft   Front-left DcMotorEx
     * @param wheel_BackRight   Back-right DcMotorEx
     * @param wheel_BackLeft    Back-left DcMotorEx
     */
    public FourWheelDrive(DcMotorEx wheel_FrontRight, DcMotorEx wheel_FrontLeft, DcMotorEx wheel_BackRight, DcMotorEx wheel_BackLeft) {
        this.wheel_FrontRight = wheel_FrontRight;
        this.wheel_FrontLeft = wheel_FrontLeft;
        this.wheel_BackRight = wheel_BackRight;
        this.wheel_BackLeft = wheel_BackLeft;

        // Set motor directions to match the physical orientation of the mecanum wheels.
        // BackLeft is reversed because it is physically mirrored on the drivetrain.
        wheel_FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheel_BackLeft.setDirection(DcMotor.Direction.REVERSE);
        wheel_FrontRight.setDirection(DcMotor.Direction.FORWARD);
        wheel_BackRight.setDirection(DcMotor.Direction.FORWARD);

        lowSensOn = false;
        lowSensMult = 1.0;          // Full power by default
        lowSensPermaMult = 0.4;     // When low sens is locked on, run at 40% power
    }

    /**
     * Standard POV-style joystick drive for mecanum wheels.
     * Left stick controls forward/back (axial) and strafe (lateral);
     * right stick X controls rotation (yaw).
     *
     * Wheel power is calculated by combining all three axes, then normalized
     * so no motor exceeds 100% power while preserving the intended direction.
     *
     * @param leftStickY  Left joystick Y axis (forward/backward)
     * @param leftStickX  Left joystick X axis (strafe left/right)
     * @param rightStickX Right joystick X axis (rotate)
     */
    public void stickDrive(double leftStickY, double leftStickX, double rightStickX) {
        double max;

        // Convert joystick inputs to robot motion axes.
        // 'direction' allows the driver to flip the robot's forward orientation mid-match.
        // Note: pushing the stick forward gives a negative Y value on the gamepad, so no inversion needed.
        double axial   = leftStickY * direction;   // Forward / backward
        double lateral = -leftStickX * direction;  // Strafe left / right
        double yaw     =  rightStickX;             // Rotation

        // For a mecanum drivetrain, each wheel gets a unique mix of axial, lateral, and yaw.
        // The signs reflect the wheel positions and roller angles.
        frontLeftPower  = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower   = axial - lateral + yaw;
        backRightPower  = axial + lateral - yaw;

        // Normalize so the highest power value is exactly 1.0 (if any exceed 1.0).
        // This prevents clipping while keeping the drive ratio between wheels intact.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Apply sensitivity multiplier to scale down speed for precision driving.
        // lowSensOn (toggled by button) uses the fixed 40% multiplier.
        // Otherwise, the dynamic lowSensMult is applied (defaults to 1.0 = full speed).
        if (lowSensOn) {
            frontLeftPower  *= lowSensPermaMult;
            frontRightPower *= lowSensPermaMult;
            backLeftPower   *= lowSensPermaMult;
            backRightPower  *= lowSensPermaMult;
        } else {
            frontLeftPower  *= lowSensMult;
            frontRightPower *= lowSensMult;
            backLeftPower   *= lowSensMult;
            backRightPower  *= lowSensMult;
        }

        // Send the computed power values to each motor
        wheel_FrontLeft.setPower(frontLeftPower);
        wheel_FrontRight.setPower(frontRightPower);
        wheel_BackLeft.setPower(backLeftPower);
        wheel_BackRight.setPower(backRightPower);
    }

    /**
     * Drives the robot in a specific field-centric direction using polar coordinates.
     * Useful for autonomous routines where you want to move at an exact angle.
     *
     * @param theta  Direction angle in radians (0 = right, PI/2 = forward)
     * @param power  Drive speed (0.0 to 1.0)
     * @param turn   Rotation component (-1.0 to 1.0)
     */
    public void directionDrive(double theta, double power, double turn) {
        // Project the direction angle onto mecanum wheel vectors (rotated 45°)
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Scale each wheel by the trig projections, then add rotation
        frontLeftPower  = power * cos / max + turn;
        frontRightPower = power * sin / max - turn;
        backLeftPower   = power * sin / max + turn;
        backRightPower  = power * cos / max - turn;

        // Normalize if total (power + rotation) would exceed 1.0
        if ((power + Math.abs(turn)) > 1) {
            frontLeftPower  /= power + Math.abs(turn);
            frontRightPower /= power + Math.abs(turn);
            backLeftPower   /= power + Math.abs(turn);
            backRightPower  /= power + Math.abs(turn);
        }

        wheel_FrontLeft.setPower(frontLeftPower);
        wheel_FrontRight.setPower(frontRightPower);
        wheel_BackLeft.setPower(backLeftPower);
        wheel_BackRight.setPower(backRightPower);
    }

    /**
     * AprilTag-assisted drive mode. Functions like stickDrive, but overrides
     * the yaw (rotation) axis to automatically center a detected AprilTag
     * in the camera frame. Useful for auto-aiming at the scoring goal.
     *
     * If no tag is detected, falls back to manual yaw control from the right stick.
     *
     * @param leftStickY  Driver left stick Y (forward/back)
     * @param leftStickX  Driver left stick X (strafe)
     * @param rightStickX Driver right stick X (manual yaw, used only when no tag is visible)
     * @param tag         The detected AprilTag to lock onto (null = no tag in frame)
     * @param camx        Camera resolution width in pixels
     * @param camy        Camera resolution height in pixels
     */
    public void lockDrive(double leftStickY, double leftStickX, double rightStickX, AprilTagDetection tag, int camx, int camy) {
        double max;
        double yaw;

        // Deadzone offset proportional to camera width — prevents jittery correction
        // when the tag is roughly centered
        double tune = camx / 55;

        // Speed at which the robot rotates to center the tag
        double yawNum = 0.5;

        double axial   = leftStickY * direction;
        double lateral = -leftStickX * direction;

        if (tag == null) {
            // No tag visible: let the driver manually control rotation
            yaw = rightStickX;
        } else {
            yaw = 0.0;

            // Tag is to the RIGHT of center: rotate clockwise to center it
            if (tag.center.x > (camx / 2) + tune) {
                double dist = (tag.center.x) - ((camx / 2) + 10.0 + tune);
                double mult = (dist / (camx / 2.0)) + 0.1;  // Scale rotation by how far off-center
                yaw = yawNum * mult;
                lastTurnDirection = 1;
            }

            // Tag is to the LEFT of center: rotate counter-clockwise to center it
            if (tag.center.x < (camx / 2) - tune) {
                double dist = ((camx / 2) + 10.0 - tune) - (tag.center.x);
                double mult = (dist / (camx / 2.0)) + 0.1;
                yaw = -yawNum * mult;
                lastTurnDirection = -1;
            }
        }

        // Combine motion axes — same mecanum math as stickDrive
        frontLeftPower  = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower   = axial - lateral + yaw;
        backRightPower  = axial + lateral - yaw;

        // Normalize to prevent exceeding 100% power
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Apply sensitivity scaling
        if (lowSensOn) {
            frontLeftPower  *= lowSensPermaMult;
            frontRightPower *= lowSensPermaMult;
            backLeftPower   *= lowSensPermaMult;
            backRightPower  *= lowSensPermaMult;
        } else {
            frontLeftPower  *= lowSensMult;
            frontRightPower *= lowSensMult;
            backLeftPower   *= lowSensMult;
            backRightPower  *= lowSensMult;
        }

        wheel_FrontLeft.setPower(frontLeftPower);
        wheel_FrontRight.setPower(frontRightPower);
        wheel_BackLeft.setPower(backLeftPower);
        wheel_BackRight.setPower(backRightPower);
    }

    /** Toggles the permanent low-sensitivity mode on or off. */
    public void toggleLowSensOn() {
        lowSensOn = !lowSensOn;
    }

    /**
     * Sets the dynamic speed multiplier (0.0–1.0).
     * Used to scale power based on trigger input for fine-grained speed control.
     */
    public void setLowSensMult(double m) {
        lowSensMult = m;
    }

    /** Returns the current dynamic speed multiplier. */
    public double getLowSensMult() {
        return lowSensMult;
    }

    /**
     * Directly sets the low sensitivity state (true = on, false = off).
     * Alternative to the toggle method for programmatic control.
     */
    public void toggleLowSens(boolean t) {
        lowSensOn = t;
    }

    // --- Telemetry getters (return last computed wheel power values) ---

    public double getFrontLeft()  { return frontLeftPower; }
    public double getFrontRight() { return frontRightPower; }
    public double getBackLeft()   { return backLeftPower; }
    public double getBackRight()  { return backRightPower; }

    /**
     * Flips the robot's driving orientation (swaps "front" and "back").
     * Useful when the robot is designed to drive from either end depending on game strategy.
     */
    public void flipDirection() {
        direction *= -1;
    }
}