package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import java.util.List;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import java.util.List;

@TeleOp(name = "TestFullBot")
public class TestFullBot extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor sweeperMotor, conveyorMotor;
    private DcMotorEx shootLeft, shootRight;
    private Servo kicker;
    private IMU imu;

    private boolean sweeperOn = false;
    private boolean conveyorOn = false;
    private boolean geckoWheelsOn = false;

    private boolean aButtonPrev = false;
    private boolean bButtonPrev = false;
    private boolean xButtonPrev = false;

    private static final int ID_BLUE_GOAL = 20;
    private static final int ID_RED_GOAL = 24;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean blueSide = true;
    private GoBildaPinpointDriver pinpoint;
    private double targetHeading = 0;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        sweeperMotor = hardwareMap.get(DcMotor.class, "sweeperMotor");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        shootLeft = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");
        kicker = hardwareMap.get(Servo.class, "kicker");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker.setPosition(0);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_trigger > 0.5) blueSide = true;
            else if (gamepad1.right_trigger > 0.5) blueSide = false;
            telemetry.addData("Side", blueSide ? "Blue" : "Red");

            if (pinpoint != null) {
                pinpoint.update();
                Pose2D pos = pinpoint.getPosition();
                telemetry.addData("Pinpoint X (mm)", pos.getX(DistanceUnit.INCH));
                telemetry.addData("Pinpoint Y (mm)", pos.getY(DistanceUnit.INCH));
                telemetry.addData("Pinpoint Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            }
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options && gamepad1.dpad_up) {
                imu.resetYaw();
                if (pinpoint != null) pinpoint.resetPosAndIMU();
            }

            double botHeading;
            if (pinpoint != null) {
                try {
                    pinpoint.update();
                    Pose2D pos = pinpoint.getPosition();
                    botHeading = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
                } catch (Exception e) {
                    botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }
            } else {
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            if (gamepad1.dpad_up) targetHeading = 0;
            else if (gamepad1.dpad_right) targetHeading = Math.toRadians(90);
            else if (gamepad1.dpad_down) targetHeading = Math.toRadians(180);
            else if (gamepad1.dpad_left) targetHeading = Math.toRadians(-90);

            double headingError = targetHeading - botHeading;
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
            double turnCorrection = headingError * 0.5;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx + turnCorrection) / denominator;
            double backLeftPower = (rotY - rotX + rx + turnCorrection) / denominator;
            double frontRightPower = (rotY - rotX - rx - turnCorrection) / denominator;
            double backRightPower = (rotY + rotX - rx - turnCorrection) / denominator;

            if (gamepad1.y) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                for (AprilTagDetection d : detections) {
                    if (d.id == (blueSide ? ID_BLUE_GOAL : ID_RED_GOAL)) {
                        double bearing = d.ftcPose.bearing;
                        double turnPower = Math.max(-0.3, Math.min(0.3, bearing / 15));
                        frontLeftPower = -turnPower;
                        backLeftPower = -turnPower;
                        frontRightPower = turnPower;
                        backRightPower = turnPower;
                        break;
                    }
                }
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad2.a && !aButtonPrev) sweeperOn = !sweeperOn;
            aButtonPrev = gamepad2.a;

            if (gamepad2.b && !bButtonPrev) conveyorOn = !conveyorOn;
            bButtonPrev = gamepad2.b;

            if (gamepad2.x && !xButtonPrev) geckoWheelsOn = !geckoWheelsOn;
            xButtonPrev = gamepad2.x;

            if (gamepad2.y) {
                shootLeft.setPower(0);
                shootRight.setPower(0);
                shootLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                shootRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (gamepad2.left_bumper) kicker.setPosition(0);
            else kicker.setPosition(0.10);

            sweeperMotor.setPower(sweeperOn ? 0.8 : 0);
            conveyorMotor.setPower(conveyorOn ? 0.5 : 0);

            if (geckoWheelsOn && gamepad2.left_bumper) {
                shootLeft.setPower(gamepad2.left_trigger);
                shootRight.setPower(0.8 * gamepad2.left_trigger);
            } else if (geckoWheelsOn) {
                shootLeft.setPower(gamepad2.left_trigger);
                shootRight.setPower(gamepad2.left_trigger);
                gamepad2.rumble(150);
            } else {
                shootLeft.setPower(0);
                shootRight.setPower(0);
            }

            telemetry.addData("Sweeper", sweeperOn ? "ON" : "OFF");
            telemetry.addData("Conveyor", conveyorOn ? "ON" : "OFF");
            telemetry.addData("Gecko Wheels", geckoWheelsOn ? "ON" : "OFF");
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.addData("Front Right Power", frontRight.getPower());
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));

            if (pinpoint != null) {
                pinpoint.update();
                // Pose2D pos = pinpoint.getPosition();
                // telemetry.addData("Pinpoint X (mm)", pos.getX());
                // telemetry.addData("Pinpoint Y (mm)", pos.y);
                // telemetry.addData("Pinpoint Heading (deg)", pos.heading);
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}
