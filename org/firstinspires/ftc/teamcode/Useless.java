
package org.firstinspires.ftc.teamcode;

import java.util.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import MRILib.util.*;

//webcam imports
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

//Limelight imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

//other
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="Spinner Test", group="Linear OpMode")

public class Useless extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx right = null;
    private DcMotorEx left = null;
        public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
     public void initAprilTag()
    { //initializing the camera and processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(aprilTag)
    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
    .setAutoStopLiveView(true)
    .build();
        setManualExposure(35, 255);
    }
    public boolean setManualExposure(int exposureMS, int gain)
    { //applying settings for the camera
        if (visionPortal == null) { return false; }
        while (!isStopRequested()
                && (visionPortal.getCameraState()
                != VisionPortal.CameraState.STREAMING))
        {
            sleep(20);
        }
        if (!isStopRequested()) 
        {
            ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) 
            {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl =
                visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        }
        return (false);
    }
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*left  = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");*/
        
        double power = .5;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        /*left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);*/

        // Wait for the game to start (driver presses START)
        waitForStart();
        initAprilTag();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            
            

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            /*left.setPower(power);
            right.setPower(power);*/
            
            /*if (gamepad1.a) power += .001;
            if (gamepad1.b) power -= .001;*/
            
            
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        
        for(AprilTagDetection detection : currentDetections) {
            telemetry.addData("AprilTag", detection.ftcPose);
            if(detection.ftcPose == null) continue;
            if(detection.robotPose == null) continue;
            double x = detection.robotPose.getPosition().x;
            double y = detection.robotPose.getPosition().y;
            double bearing = detection.ftcPose.bearing;
            double yaw = detection.ftcPose.yaw;
            double aprilTagX = detection.ftcPose.x;
            double aprilTagY = detection.ftcPose.y;
            telemetry.addData("",detection.ftcPose.bearing-detection.ftcPose.yaw);
            //blue goal
            double heading = detection.ftcPose.bearing - detection.ftcPose.yaw;
            double Y = y * Math.cos(heading) - x * Math.sin(heading);
            double X = y * Math.sin(heading) + x * Math.cos(heading);
            X = aprilTagX - x;
            Y = aprilTagY - y;
        
            telemetry.addData("Position", X+" "+Y);
            
            telemetry.addData("position", detection.ftcPose.x+" "+detection.ftcPose.y);
        }
            
            /*telemetry.addData("power", power);
            telemetry.addData("lvel", left.getVelocity());
            telemetry.addData("rvel", right.getVelocity());*/
            

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
    
}
