package MRILib.motion;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import MRILib.managers.*;
import MRILib.util.Mathf;

/*
 * This file manages the three pid equations made at init for x, y, and turning movements respectively
 * It uses the calculated power output by each pid in order to determine power needed
 * on each motor to move in a strait line to the target position. Instructions on how to tune the PIDs
 * can be found in the example Autonomous file
 */


public class PIDController {
    private Bot bot = null;
    private Telemetry telemetry;

    public PID xPID;
    public PID yPID;
    public PID thetaPID;

    public double targetX = 0;
    public double targetY = 0;
    public double targetRadians = 0; //radians
    
    public Supplier<Double> maxAngSpeed = ()-> .7; //.85
    public Supplier<Double> maxSpeed; //.9
    public double POWER_SCALE = 1;
    
    
    ElapsedTime timer = null;
    
    public boolean nextState = false;
    
    public PIDController(Bot bot, Telemetry telemetry){
        maxSpeed = ()->.8;
        this.bot = bot;
        this.telemetry = telemetry;
    }

    public void setPID(PID x, PID y){
        xPID = x;
        yPID = y;
    }
    public void setTurnPID(PID theta){
        thetaPID = theta;
    }

    public void start(){
        xPID.start();
        yPID.start();
        thetaPID.setTarget(0);
    }

    public void update() {
        // Get the current pose and heading (field-centric)
        Pose2D curPos = bot.getPosition();
        double curX = curPos.getX(DistanceUnit.INCH);
        double curY = curPos.getY(DistanceUnit.INCH);
        double botHeading = Math.toRadians(bot.getHeading());  // current heading in radians
    
        // Get field-centric PID outputs for x and y (translation)
        double xVal = xPID.update(curX);  // how far we are from target X
        double yVal = -yPID.update(curY);  // how far we are from target Y
        
        double thetaVal = -thetaPID.update(botHeading);
        thetaVal = Range.clip(thetaVal, -1.0, 1.0);
        
        double max = Math.max(xVal, yVal);
        
        //countering stalling
        // if(Math.abs(xVal)>.1)xVal+=.07*Math.signum(xVal);
        // if(Math.abs(yVal)>.1)yVal+=.07*Math.signum(yVal);
        // if(Math.abs(thetaVal)>.1)thetaVal+=.1*Math.signum(thetaVal);
        
        // clamping translation while keeping ratio and
        // sending field centric power call to be translated to robot centric
        telemetry.addData("thetaval", thetaVal);
        if(max>1)
            bot.driveFieldXYW(yVal/max*maxSpeed.get(), xVal/max*maxSpeed.get(), thetaVal*maxAngSpeed.get());
        else
            bot.driveFieldXYW(yVal*maxSpeed.get(), xVal*maxSpeed.get(), thetaVal*maxAngSpeed.get());
        //bot.driveFieldXYW(0,0,thetaVal*maxAngSpeed);
        
        
            
        // telemetry.addData("CurX", curX);
        // telemetry.addData("targetX", xPID.targetVal);
        // telemetry.addData("CurY", curY);
        // telemetry.addData("targetY", yPID.targetVal);
        // telemetry.addData("CurTheta", botHeading);
        // telemetry.addData("targetTheta", thetaPID.targetVal);
        // telemetry.addLine();
        // telemetry.addData("xVal", xVal);
        // telemetry.addData("yVal", yVal);
        // telemetry.addData("thetaVal", thetaVal);
        // telemetry.addData("xVal Scaled", xVal/max*maxSpeed);
        // telemetry.addData("yVal Scaled", yVal/max*maxSpeed);
        // telemetry.addData("thetaVal Scaled", thetaVal);
        
        
    
    
    
    
    
    
        // // Optionally: Set PID output to 0 if very close to the target to avoid overshoot.
        // if (Math.abs(xPID.targetVal - curX) < 4) xVal = 0;
        // if (Math.abs(yPID.targetVal - curY) < 4) yVal = 0;
    
        // Convert field-centric to robot-centric using the current heading
        // double robotX =  xVal * Math.cos(-botHeading) - yVal * Math.sin(-botHeading);
        // double robotY =  xVal * Math.sin(-botHeading) + yVal * Math.cos(-botHeading);
    
        // // Calculate robot's actual movement speed (clamp to max speed)
        // double robotSpeed = Math.hypot(robotX, robotY);
        // if (robotSpeed > maxSpeed) {
        //     double scale = maxSpeed / robotSpeed;  // scale if the speed exceeds max
        //     robotX *= scale;
        //     robotY *= scale;
        // }
    
        // // Standard mecanum drive calculations (no inputTurn yet)
        // double frontLeft = robotX + robotY;
        // double frontRight = robotX - robotY;
        // double backLeft = robotX - robotY;
        // double backRight = robotX + robotY;
    
        // // // Normalize the motor powers to avoid overpowering
        // // double maxMotorPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
        // //                                 Math.max(Math.abs(backLeft), Math.abs(backRight)));
        
        // // if (maxMotorPower > 1) {
        // //     frontLeft /= maxMotorPower;
        // //     frontRight /= maxMotorPower;
        // //     backLeft /= maxMotorPower;
        // //     backRight /= maxMotorPower;
        // // }
    
        // // Apply the motor powers to the robot
        // bot.getFL().setPower(frontLeft * POWER_SCALE);
        // bot.getFR().setPower(frontRight * POWER_SCALE);
        // bot.getBL().setPower(backLeft * POWER_SCALE);
        // bot.getBR().setPower(backRight * POWER_SCALE);
    
        // // Telemetry data (for debugging)
        // telemetry.addData("curX", curX);
        // telemetry.addData("curY", curY);
        // telemetry.addData("xVal", xVal);
        // telemetry.addData("yVal", yVal);
        // telemetry.addData("robotX", robotX);
        // telemetry.addData("robotY", robotY);
        // telemetry.addData("robotSpeed", robotSpeed);
        // telemetry.addData("frontLeft", frontLeft);
        // telemetry.addData("frontRight", frontRight);
        // telemetry.addData("backLeft", backLeft);
        // telemetry.addData("backRight", backRight);
    }





    // sets the target position to move towards
    public void moveTo(double x, double y, double degrees) {
        xPID.setTarget(x);
        targetX = x;
        yPID.setTarget(y);
        targetY = y;
        targetRadians = Math.toRadians(degrees);
        thetaPID.setTarget(targetRadians);
    }

    public void setAngle(double degrees) {
        targetRadians = Math.toRadians(degrees);
        thetaPID.setTarget(targetRadians);
    } 

    public void setSpeeds(double strafe, double ang){
        setMaxSpeed(strafe);
        setMaxAngSpeed(ang);
    }
    public void setMaxSpeed(double s){
        maxSpeed = ()-> s;
    }
    public void setMaxAngSpeed(double s){
        maxAngSpeed = ()-> s;
    }


    // getters
    public double getTargetX(){
        return targetX;
    }
    public double getTargetY(){
        return targetY;
    }
    public double getTargetRadians(){
        return targetRadians;
    }
    public double getTargetDegrees(){
        return -Math.toDegrees(targetRadians);
    }
    public double getCurrentDegrees(){
        return Math.toDegrees(Math.toRadians(-bot.getPosition().getHeading(AngleUnit.DEGREES)));
    }
    public double getCurrentRadians(){
        return Math.toRadians(-bot.getPosition().getHeading(AngleUnit.DEGREES));
    }
    
    
}
