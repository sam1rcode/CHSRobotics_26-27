package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "ControlMode", group = "Robot")
public class ControlModeRed extends LinearOpMode {

    // Match timer — used for debouncing and elapsed-time telemetry
    private ElapsedTime runtime = new ElapsedTime();
    
    /** Records the current time as the last-pressed time for a given button. */
    public void setButtonTime(String button) {
        buttonTimes.put(button, runtime.seconds());
    }

    /** Returns the last time a given button was pressed (defaults to 0 if never pressed). */
    public double getButtonTime(String button) {
        buttonTimes.putIfAbsent(button, 0.0);
        return buttonTimes.get(button);
    }

    public boolean debounce(String button, double time) {
        return runtime.seconds() - getButtonTime(button) > time;
    }

    @Override
    public void runopMode() {

        FourWheelDrive wheels = new FourWheelDrive(
            hardwareMap.get(DcMotorEx.class, "wheel_FR"),
            hardwareMap.get(DcMotorEx.class, "wheel_FL"),
            hardwareMap.get(DcMotorEx.class, "wheel_BR"),
            hardwareMap.get(DcMotorEx.class, "wheel_BL")
        );

        waitForStart();   // Wait for referee to press START
        runtime.reset();

        while(opModeIsActive()){

            wheels.stickDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", wheels.getFrontLeft(), wheels.getFrontRight());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", wheels.getBackLeft(), wheels.getBackRight());
            telemetry.update();

        }

    }

}