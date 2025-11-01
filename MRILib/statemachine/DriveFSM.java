package MRILib.statemachine;

import MRILib.managers.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import MRILib.motion.PIDController;
import MRILib.motion.PID;
import MRILib.util.Mathf;
import MRILib.eventlistener.BotEventManager.*;
import MRILib.eventlistener.BotEventManager;
import MRILib.eventlistener.BotEvent;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.concurrent.atomic.AtomicReference;

public class DriveFSM
{
    private Bot bot;
    // private PIDController pid = null;
    public PID xPid;
    public PID yPid;
    public PID wPid;
    
    public ArrayList<BotState> steps = new ArrayList<>();
    Telemetry telemetry;

    public int currentStep = 0;

    public DriveFSM(Bot bot, PID xPid, PID yPid, PID wPid, Telemetry telemetry){
        this.bot = bot;
        this.xPid = xPid;
        this.yPid = yPid;
        this.wPid = wPid;
        this.telemetry = telemetry;
    }
    
    public void start(Pose2D startPos){
        lastX = startPos.getX(DistanceUnit.INCH);
        lastY = startPos.getY(DistanceUnit.INCH);
        lastAngle = startPos.getHeading(AngleUnit.DEGREES);
        start();
    }
    
    public void start()
    { // runs once at the state's initialization
        if(steps.size()>currentStep)steps.get(currentStep).start();
    }

    public void update()
    { // runs every repeat loop on the main thread
        if(steps.size()>currentStep)steps.get(currentStep).update();
    }

    public void end()
    { // runs once at the end of the state or before transitioning to next state
        if(steps.size()>currentStep)steps.get(currentStep).end();
    }

    public void nextState()
    { // transitioning to the next state once target condition is met
        if (currentStep > steps.size()) return;
        end();
        currentStep++;
        start();
    }

    //MOVETO OVERLOADS
    
    public DriveFSM moveTo(double x, double y, double theta, Runnable command) {
        return moveTo(x, y, theta, command, 6); }

    public DriveFSM moveTo(double x, double y, double theta, double timeout) { 
        return moveTo(x, y, theta, null, timeout); }

    public DriveFSM moveTo(double x, double y, double theta) { 
        return moveTo(x, y, theta, null, 6); }

    public DriveFSM moveTo(Pose2D pose){ 
        return moveTo(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES)); }

    //MOVETO BASE METHOD
    
    public double lastX = 0;
    public double lastY = 0;
    public double lastAngle = 0;
    
    // Adding a new state to the step list to move to a position
    public DriveFSM moveTo(double x, double y, double theta, Runnable command, double timeout) {
        steps.add(currentStep==0?steps.size():currentStep+1,
            new BotState("MOVE_TO:: (" + x + ", " + y + ") " + theta + "°")
        { // Overriding the base methods of BotState with logic specific to this state
            double tx = x;
            double ty = y;
            ElapsedTime timer;

            double tAngle = theta;
            
            @Override
            void start(){
                // Running parallel arm control
                if(command!=null)command.run();

                // Setting PID target position
                // pid.moveTo(tx,ty,tAngle);
                xPid.setTarget(tx);
                yPid.setTarget(ty);
                wPid.setTarget(tAngle);
    
                // Saving old values
                lastX = tx;
                lastY = ty;
                lastAngle = tAngle;

                // Starting timer for timeout
                timer = new ElapsedTime();
            }
            @Override
            void update(){
                // Updating pid to set motor values
                // pid.setAngle(tAngle);
                double fx = xPid.update(bot.getX());
                double fy = yPid.update(bot.getY());
                double fw = wPid.update(bot.getHeading());
                
                // Set power to robot
                bot.driveFieldXYW(fy, -fx, fw);

                // Calculating distance from target position using odometry
                Pose2D curPos = bot.getPosition();
                double deltaX = tx - curPos.getX(DistanceUnit.INCH);
                double deltaY = ty - curPos.getY(DistanceUnit.INCH);
                double deltaAngle = bot.getHeading()-tAngle;
                //deltaAngle = deltaAngle;
                double dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                telemetry.addData("error", dist);
                telemetry.addData("radiansError", Math.toRadians(deltaAngle));
                telemetry.addData("thetaError", deltaAngle);

                // Declaring the acceptable error to allow moving onto the next step
                if(dist < 5 && Math.abs(deltaAngle) < 3){
                    nextState();
                }
                // Moving onto the next step if timer is above a timeout
                //      Mostly used to not get stuck indefinitely in the case of odometry not hitting its target
                //      or robot getting stuck on something
                if(timer.seconds() > timeout)
                    nextState();
                
            }
        });
        return this;
    }

    //WAITFORSECONDS OVERLOADS

    public DriveFSM waitForSeconds(double seconds){
        return waitForSeconds(seconds, null); }

    //WAITFORSECONDS BASE METHOD

    public DriveFSM waitForSeconds(double seconds, Runnable command)
    { // Adding a new state to the step list to wait for a specified time
        steps.add(new BotState("WAIT_FOR_SECONDS:: (" + seconds + ")")
        { // Overriding botstate with wait logic
            ElapsedTime timer;

            @Override
            void start(){
                // Running parallel arm command
                if(command!=null)command.run();

                // Subscribe to overriding of angle
                
                // Setting pid target to the last target
                //pid.moveTo(lastX,lastY,lastAngle);
                bot.driveXYW(0,0,0);
                timer = new ElapsedTime();
            }
            @Override
            void update(){
                // Updating pid to move back to rest position if pushed out of it
                //pid.update();

                // Moving to next state if wait value has elapsed
                if(timer.seconds() > seconds) nextState();
            }
        });
        return this;
    }
    
    public DriveFSM run(Runnable command){
        steps.add(new BotState("RUN:: " + command){
            @Override void start(){
                if(command!=null)command.run();
            }
            @Override
            void update(){
                nextState();
            }
        });
        return this;
    }
}