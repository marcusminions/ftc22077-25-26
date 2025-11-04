package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

@TeleOp(name = "KrishTestApril")
public class KrishAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    
    private static final int ID_BLUE_GOAL = 20; 
    private static final int ID_RED_GOAL = 24; 
    
    private static final int ID_OBELISK_1 = 21;
    private static final int ID_OBELISK_2 = 22;
    private static final int ID_OBELISK_3 = 23;

    private static final String ART_G = "G";
    private static final String ART_P = "P";

    // Camera positions for the thing "localization" - Fancy ahhh word
    // Measure from center of drivetrain
    // X (right side), Y (Forward), Z (Vertical) if ya dont know
    private Position camPos = new Position(DistanceUnit.INCH, 
            0.0, // X: Inches right of robot center 
            0*-7.75, // Y: Inches forward of robot center
            0*5.5, // Z: Inches up from the floor
            0); // Timestamp (0 is fine here)
    
    // Yaw, Pitch, Roll - relative to robots forward position
    // Yaw: Rotation around Z axis, 0=forward
    // Pitch: Rotation around X axis, 0=level
    // Roll: Rotation around Y axis, 0=level
    private YawPitchRollAngles camOrient = new YawPitchRollAngles(AngleUnit.DEGREES, 
            0.0, // Yaw: Rotation from robots forward direction
            0*20.0, // Pitch: -90 is camera pointing straight down, 0 if is horizontal
            0.0, // Roll: the tilt side to side
            0); // Js keep zero here idk what an error was and GPT told me this fixes it which it did

    private AprilTagProcessor aprilTag; 
    private VisionPortal visionPortal; 

    @Override 
    public void runOpMode() { 
        initAprilTag(); 

        telemetry.addData("Status", "On"); 
        telemetry.update(); 
        waitForStart(); 

        while (opModeIsActive()) { 
            processAprilTags(); 
            telemetry.update(); 
            
            if (gamepad1.dpad_down) { 
                visionPortal.stopStreaming(); 
            } else if (gamepad1.dpad_up) { 
                visionPortal.resumeStreaming(); 
            } 
            
            sleep(20); 
        } 
        visionPortal.close(); 
    } 

    private void initAprilTag() { 

        aprilTag = new AprilTagProcessor.Builder() 
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary()) 
                .setCameraPose(camPos, camOrient) 
                .build(); 

        VisionPortal.Builder builder = new VisionPortal.Builder(); 

        if (USE_WEBCAM) { 
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); 
        }

        builder.addProcessor(aprilTag); 

        visionPortal = builder.build(); 
    } 

    private void processAprilTags() { 

        List<AprilTagDetection> detections = aprilTag.getDetections(); 
        telemetry.addData("# Tags", detections.size()); 
        
        for (AprilTagDetection detection : detections) { 
            if (detection.metadata != null) { 
                int id = detection.id;
                
                if (id == ID_BLUE_GOAL || id == ID_RED_GOAL) {
                    
                    telemetry.addLine(String.format("\nGOAL (ID %d) - POSITION", id)); 
                    
                    
                    
                    telemetry.addData("Pos (inch) XYZ", 
                            String.format("%6.1f, %6.1f, %6.1f", 
                            detection.robotPose.getPosition().x, 
                            detection.robotPose.getPosition().y, 
                            detection.robotPose.getPosition().z)); 
                    
                    
                    
                    telemetry.addData("Orientation (deg) PRY",
                            String.format("%6.1f, %6.1f, %6.1f",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES), 
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES), 
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                } 
                
                else if (id == ID_OBELISK_1 || id == ID_OBELISK_2 || id == ID_OBELISK_3) {
                    
                    telemetry.addLine(String.format("\nOBELISK (ID %d)", id)); 
                    
                    String motif;
                    if (id == ID_OBELISK_1) {
                        motif = ART_G + ART_P + ART_P;
                    } else if (id == ID_OBELISK_2) {
                        motif = ART_P + ART_G + ART_P;
                    } else { 
                        motif = ART_P + ART_P + ART_G;
                    }

                    
                    
                    String pattern = motif + motif + motif;
                    telemetry.addData("Motif (3)", motif);
                    telemetry.addData("Pattern (9)", pattern);

                    
                    
                    telemetry.addData("Range/Bearing (ftcPose)",
                            String.format("R: %6.1f, B: %6.1f deg",
                            detection.ftcPose.range,
                            detection.ftcPose.bearing));

                } else {
                    
                    telemetry.addLine(String.format("\nOTHER TAG (ID %d)", id));
                }
            } else { 
                
                telemetry.addLine(String.format("\nUnknown Tag (ID %d)", detection.id)); 
                telemetry.addData("Center (pixels)", 
                    String.format("X: %6.0f, Y: %6.0f", detection.center.x, detection.center.y)); 
            } 
        } 
        
        telemetry.addLine("\nKey:\nXYZ = X, Y, Z dist."); 
        telemetry.addLine("PRY = Pitch, Roll, Yaw"); 
    } 
}

