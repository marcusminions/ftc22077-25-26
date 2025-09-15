package org.firstinspires.ftc.teamcode;

import java.util.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class LimeLightTest extends LinearOpMode {
    private Limelight3A limelight;
    
    private final int h = 12; //inches
    private final int LLAngle = 65; //degrees
    
    private final double clawOffset_x = 1;
    private final double clawOffset_y = 5;
    
    DetectorResult target = null;
    
    

    @Override
    public void runOpMode() {
        
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        limelight.start();

        double minPower = 0.2;
        double maxPower = 0.4;
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if(result != null){
                target = result.getDetectorResults().get(0);
                double xPower = 0;
                double yPower = 0;
                
                
                if(gamepad2.right_trigger>0){
                    telemetry.addData("detection", target);
                    if(target!=null){
                        double targetX = target.getTargetXDegrees();
                        xPower = (double)(Math.abs(targetX)-minPower)/(maxPower-minPower);
                        double targetY = target.getTargetYDegrees();
                        yPower = (double)(Math.abs(targetY)-minPower)/(maxPower-minPower);
                        
                        xPower*=Math.signum(targetX);
                        yPower*=Math.signum(targetY);
                    }
                }
                
                // try{
                //     List<AprilTagDetection> currentDetections = bot.aprilTag.getDetections();
                //     for (AprilTagDetection detection : cumDetections) {
                //         //set odomentry
                //         double offset = 5.75;
                //         double range = detection.ftcPose.range;
                //         double bearing = detection.ftcPose.bearing;
                //         double yaw = detection.ftcPose.yaw;
                //         double tagx = detection.metadata.fieldPosition.get(0);
                //         double tagy = detection.metadata.fieldPosition.get(1);
                //         double theta = Math.toRadians(-(bot.getHeading() + bearing));
                //         double fx = tagx - Math.cos(theta) * range;
                //         double fy = tagy - Math.sin(theta) * range;
                        
                //         double offsetX = Math.cos(Math.toRadians(bot.getHeading())) * offset;
                //         double offsetY = Math.sin(Math.toRadians(bot.getHeading())) * offset;
                //         double fieldX = fx + offsetX;
                //         double fieldY = fy + offsetY;
                //         //odo.setFieldXY(fx, fy);
                //         telemetry.addData("april tag x", fx);
                //         telemetry.addData("april tag y", fy);
                //         telemetry.addData("field x", fieldX);
                //         telemetry.addData("field y", fieldY);
                //         telemetry.addLine();
                //         //bot.setPosition(fieldX, fieldY, bot.getHeading());
                //     }
                // }
                // catch( Exception e){
                    
                // }
                telemetry.addData("xPower", xPower);
                telemetry.addData("yPower", yPower);
                telemetry.addData("isrunning", limelight.isRunning());
            }
            
            telemetry.update();
        }
    }
    
    double getHeading(){
        return 0;
    }
    
    // public Pose2D calculateSamplePosition(DetectorResult target)
    // {
    //     double ty = target.getTargetYDegrees();
    //     double tx = target.getTargetXDegrees();
        
    //     double hypot = h / Math.cos(Math.toRadians(LLAngle+ty));
    //     double rx = h * Math.tan(Math.toRadians(LLAngle+ty));
    //     double ry = hypot * Math.tan(Math.toRadians(tx));
        
    //     double hypot2 = Math.sqrt(rx*rx + ry*ry);
    //     double fx = hypot2 * Math.cos(Math.toRadians(getHeading()+tx));
    //     double fy = hypot2 * Math.sin(Math.toRadians(getHeading()+tx));
        
    //     List<List<Double>> corners = target.getTargetCorners();
    //     height = corners.get(0).get(0)-corners.get(1).get(0);
    //     width = corners.get(0).get(0)-corners.get(0).get(1);
    //     double targetTheta = (height>width?0:90) + getHeading();
        
    //     return new Pose2D(DistanceUnit.INCH, fx, fy, AngleUnit.DEGREES, targetTheta);
    // }
    
    // public Pose2D calculateTargetPosition(Pose2D botPos, Pose2D samplePos)
    // {
    //     double radius = Math.hypot(clawOffset_x, clawOffset_y); //distance from sample to pickup
    //     double dirX = botPos.getX(DistanceUnit.INCH) - samplePos.getX(DistanceUnit.INCH);
    //     double dirY = botPos.getY(DistanceUnit.INCH) - samplePos.getY(DistanceUnit.INCH);
    //     double dirMag = Math.hypot(dirX, dirY);
    //     dirX/=dirMag;
    //     dirY/=dirMag;
        
    //     double targetX = samplePos.getX(DistanceUnit.INCH) + radius * dirX;
    //     double targetY = samplePos.getY(DistanceUnit.INCH) + radius * dirY;
        
    //     double headingRadians = Math.atan2(samplePos.getY(DistanceUnit.INCH)-targetY, samplePos.getX(DistanceUnit.INCH)-targetX);
        
    //     return new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.RADIANS, headingRadians);
        
    // }
    
}
