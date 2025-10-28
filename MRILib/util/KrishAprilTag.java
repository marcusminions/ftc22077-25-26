package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

public class KrishAprilTag {

    private static final boolean USE_WEBCAM = true;

    private static final int ID_BLUE_GOAL = 20;
    private static final int ID_RED_GOAL = 24;
    private static final int ID_OBELISK_1 = 21;
    private static final int ID_OBELISK_2 = 22;
    private static final int ID_OBELISK_3 = 23;

    private static final String ART_G = "G";
    private static final String ART_P = "P";

    private final LinearOpMode opMode;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private final Position camPos = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);
    private final YawPitchRollAngles camOrient = new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0);

    public KrishAprilTag(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(camPos, camOrient)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void processAprilTags() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        opMode.telemetry.addData("# Tags", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                int id = detection.id;

                if (id == ID_BLUE_GOAL || id == ID_RED_GOAL) {
                    opMode.telemetry.addLine(String.format("\nGOAL (ID %d) - POSITION", id));
                    opMode.telemetry.addData("Pos (inch) XYZ",
                            String.format("%6.1f, %6.1f, %6.1f",
                                    detection.robotPose.getPosition().x,
                                    detection.robotPose.getPosition().y,
                                    detection.robotPose.getPosition().z));
                    opMode.telemetry.addData("Orientation (deg) PRY",
                            String.format("%6.1f, %6.1f, %6.1f",
                                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                } else if (id == ID_OBELISK_1 || id == ID_OBELISK_2 || id == ID_OBELISK_3) {
                    opMode.telemetry.addLine(String.format("\nOBELISK (ID %d)", id));

                    String motif;
                    if (id == ID_OBELISK_1) motif = ART_G + ART_P + ART_P;
                    else if (id == ID_OBELISK_2) motif = ART_P + ART_G + ART_P;
                    else motif = ART_P + ART_P + ART_G;

                    String pattern = motif + motif + motif;
                    opMode.telemetry.addData("Motif (3)", motif);
                    opMode.telemetry.addData("Pattern (9)", pattern);
                    opMode.telemetry.addData("Range/Bearing (ftcPose)",
                            String.format("R: %6.1f, B: %6.1f deg",
                                    detection.ftcPose.range,
                                    detection.ftcPose.bearing));
                } else {
                    opMode.telemetry.addLine(String.format("\nOTHER TAG (ID %d)", id));
                }
            } else {
                opMode.telemetry.addLine(String.format("\nUnknown Tag (ID %d)", detection.id));
                opMode.telemetry.addData("Center (pixels)",
                        String.format("X: %6.0f, Y: %6.0f", detection.center.x, detection.center.y));
            }
        }

        opMode.telemetry.addLine("\nKey:\nXYZ = X, Y, Z dist.");
        opMode.telemetry.addLine("PRY = Pitch, Roll, Yaw");
    }

    public void stopStream() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    public void resumeStream() {
        if (visionPortal != null) visionPortal.resumeStreaming();
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }
    public List<AprilTagDetection> getDetections() {
    return aprilTag.getDetections();
    }
}
