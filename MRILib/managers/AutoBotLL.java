package MRILib.managers;

import java.util.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;

import MRILib.util.*;
import MRILib.GameValues.COLOR;
import static MRILib.BotValues.*;

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
import com.qualcomm.robotcore.hardware.HardwareMap;


public class AutoBotLL extends LaunchBot {
    
    public Limelight3A limelight;
    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean started = false;
    private DetectorResult lastDetection = null;
    
    public AutoBotLL(LinearOpMode op){
        super(op);
        initLL();
        initAprilTag();
    }
    
    
    private final double LL_height = 8.6; //inches
    private final double LL_angle = Math.toRadians(65); //radians
    
    private final double LL_offsetX = 6.5; //inches
    private final double LL_offsetY = 4.75;
    private final double claw_offsetX = 0;
    private final double claw_offsetY = 14.5;
    
    public void initLL(){
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
    }

    public void initAprilTag()
    { //initializing the camera and processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            op.hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        setManualExposure(35, 255);
    }
    
    public void start(){
        limelight.start();
        limelight.pipelineSwitch(1);
    }
    
    public void update(){
        super.update();
    }
    
    public LLResult getLLResult(){
        return limelight.getLatestResult();
    }
    
    public DetectorResult getLLDetection(){
        LLResult result = limelight.getLatestResult();
        DetectorResult target = null;
        
        //parsing all targets and choosing the best one
        if(result!=null && result.isValid() && result.getPipelineIndex()==1){
            List<DetectorResult> detections = result.getDetectorResults();
            if(detections!=null && detections.get(0)!=null)lastDetection = detections.get(0);
        }
        
        return lastDetection;
    }
    
    public boolean isDetectionValid(){
        return lastDetection!=null;
    }
    
    public static String detectionToString(DetectorResult detection){
        if(detection==null) return "null object reference";
        return "X: "+detection.getTargetXDegrees() + " | Y: "+detection.getTargetYDegrees();
    }
    
    public void stopLL(){
        limelight.stop();
    }

    public boolean setManualExposure(int exposureMS, int gain)
    { //applying settings for the camera
        if (visionPortal == null) { return false; }
        while (!op.isStopRequested()
                && (visionPortal.getCameraState()
                != VisionPortal.CameraState.STREAMING))
        {
            op.sleep(20);
        }
        if (!op.isStopRequested()) 
        {
            ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) 
            {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                op.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            op.sleep(20);
            GainControl gainControl =
                visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            op.sleep(20);
            return (true);
        }
        return (false);
    }
    
    public void getPositionWebcam(AprilTagDetection detection) {
        if(detection.id == 20 || detection.id == 24) {
            //goal
            double x = detection.robotPose.getPosition().x - WEBCAM_X_OFFSET;
            double y = detection.robotPose.getPosition().y - WEBCAM_Y_OFFSET;
            double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) - WEBCAM_ANGLE_OFFSET;
            
            Pose2D position = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw);
            
            setPosition(x, y, yaw);
            
        }
        else if(detection.id == 21 || detection.id == 22 || detection.id == 23) {
            //obelisk
            op.telemetry.addData("Obelisk Apriltag with id", detection.id);
        }
        else {
            op.telemetry.addData("unknown apriltag with id", detection.id);
            
        }
    }
    
    public COLOR[] getPatternWebcam(AprilTagDetection detection) {
        if(detection.id == 21 || detection.id == 22 || detection.id == 23) {
            if(detection.id == 21) {
                //GPP
                return new COLOR[]{COLOR.GREEN, COLOR.PURPLE, COLOR.PURPLE};
            }
            else if(detection.id == 22) {
                //PGP
                return new COLOR[]{COLOR.PURPLE, COLOR.GREEN, COLOR.PURPLE};
            }
            else if(detection.id == 23) {
                //PPG
                return new COLOR[]{COLOR.PURPLE, COLOR.PURPLE, COLOR.GREEN};
            }
        }
        else if (detection.id == 20 || detection.id == 24) {
            op.telemetry.addData("Goal Apriltag with id", detection.id);
        }
        else {
            op.telemetry.addData("Unknown AprilTag with id", detection.id);
        }
        return null;
    }
    
}
