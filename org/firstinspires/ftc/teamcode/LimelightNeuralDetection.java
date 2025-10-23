package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "LimelightNeuralDetection", group = "Sensor")
public class LimelightNeuralDetection extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private enum SelectionMode { CLOSEST, FURTHEST }
    private enum ColorFilter { ANY, GREEN, PURPLE }

    private SelectionMode currentMode = SelectionMode.CLOSEST;
    private ColorFilter currentColorFilter = ColorFilter.ANY;

    private boolean leftTriggerPressed = false;
    private boolean rightTriggerPressed = false;

    // Tune the fucking value If it tweaks like ailani
    // Start with 0.015, If the robot turns too slowly increase the fucking number. 
    //n If it overshoots decrease the fucking value.
    private static final double TURN_KP = 0.005;
    private static final double MAX_TURN_SPEED = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(7);
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("INIT");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleGamepadInputsForTargeting();
            
            double y = -gamepad1.left_stick_y; 
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            LLResult result = limelight.getLatestResult();
            LLResultTypes.DetectorResult target = null;
            if (result != null && result.isValid() && result.getDetectorResults() != null && !result.getDetectorResults().isEmpty()) {
                target = findBestTarget(result.getDetectorResults());
            }

            double turnPower = rx; 

            if (gamepad1.a && target != null) {
                double error = target.getTargetXDegrees();
                turnPower = Range.clip(error * TURN_KP, -MAX_TURN_SPEED, MAX_TURN_SPEED);
                telemetry.addData("Mode", "AUTO-AIM");
            } else {
                telemetry.addData("Mode", "MANUAL / FIELD-CENTRIC");
            }

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnPower), 1);
            double frontLeftPower = (rotY + rotX + turnPower) / denominator;
            double backLeftPower = (rotY - rotX + turnPower) / denominator;
            double frontRightPower = (rotY - rotX - turnPower) / denominator;
            double backRightPower = (rotY + rotX - turnPower) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);


            updateTelemetry(target);
        }

        limelight.stop();
    }

    private void handleGamepadInputsForTargeting() {
        if (gamepad1.left_trigger > 0.5 && !leftTriggerPressed) {
            leftTriggerPressed = true;
            if (currentMode == SelectionMode.CLOSEST) {
                currentMode = SelectionMode.FURTHEST;
            } else {
                currentMode = SelectionMode.CLOSEST;
            }
        } else if (gamepad1.left_trigger < 0.5) {
            leftTriggerPressed = false;
        }

        if (gamepad1.right_trigger > 0.5 && !rightTriggerPressed) {
            rightTriggerPressed = true;
            if (currentColorFilter == ColorFilter.ANY) {
                currentColorFilter = ColorFilter.GREEN;
            } else if (currentColorFilter == ColorFilter.GREEN) {
                currentColorFilter = ColorFilter.PURPLE;
            } else {
                currentColorFilter = ColorFilter.ANY;
            }
        } else if (gamepad1.right_trigger < 0.5) {
            rightTriggerPressed = false;
        }
    }

    private LLResultTypes.DetectorResult findBestTarget(List<LLResultTypes.DetectorResult> detections) {
        LLResultTypes.DetectorResult bestTarget = null;
        double bestValue = 0;

        List<LLResultTypes.DetectorResult> filteredDetections = new ArrayList<>();
        if (currentColorFilter == ColorFilter.ANY) {
            filteredDetections.addAll(detections);
        } else {
            String targetColor = (currentColorFilter == ColorFilter.GREEN) ? "green" : "purple";
            for (LLResultTypes.DetectorResult detection : detections) {
                if (detection.getClassName().equalsIgnoreCase(targetColor)) {
                    filteredDetections.add(detection);
                }
            }
        }

        if (filteredDetections.isEmpty()) {
            return null;
        }

        for (LLResultTypes.DetectorResult detection : filteredDetections) {
            double area = detection.getTargetArea();

            if (bestTarget == null) { 
                bestTarget = detection;
                bestValue = area;
            } else {
                if (currentMode == SelectionMode.CLOSEST) { 
                    if (area > bestValue) {
                        bestValue = area;
                        bestTarget = detection;
                    }
                } else { 
                    if (area < bestValue) {
                        bestValue = area;
                        bestTarget = detection;
                    }
                }
            }
        }
        return bestTarget;
    }

    private void updateTelemetry(LLResultTypes.DetectorResult target) {
        telemetry.addData("Selection Mode", currentMode);
        telemetry.addData("Color Filter", currentColorFilter);

        if (target != null) {
            telemetry.addLine("--- TARGET LOCKED ---")
                    .addData("Label", target.getClassName())
                    .addData("X-Offset", "%.2f°", target.getTargetXDegrees());
        } else {
            telemetry.addLine("--- NO TARGET ---");
        }
        telemetry.update();
    }
}


