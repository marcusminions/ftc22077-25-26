package MRILib.statemachine;

import MRILib.managers.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import MRILib.motion.PIDController;
import MRILib.util.Mathf;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class DriveFSM
{
    private Bot bot;
    private PIDController pid = null;
    ArrayList<BotState> steps = new ArrayList<>();
    Telemetry telemetry;

    public int currentStep = 0;

    public DriveFSM(Bot bot, PIDController pid, Telemetry telemetry){
        this.bot = bot;
        this.pid = pid;
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
        return moveTo(x, y, theta, command, 5); }

    public DriveFSM moveTo(double x, double y, double theta, double timeout) { 
        return moveTo(x, y, theta, null, timeout); }

    public DriveFSM moveTo(double x, double y, double theta) { 
        return moveTo(x, y, theta, null, 3); }

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
            double tAngle = theta;
            ElapsedTime timer;
            
            @Override
            void start(){
                // Running parallel arm control
                if(command!=null)command.run(); 

                // Setting PID target position
                pid.moveTo(tx,ty,tAngle);

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
                pid.update();

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
                if(dist < 3 && Math.abs(deltaAngle) < 3){
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

                // Setting pid target to the last target
                //pid.moveTo(lastX,lastY,lastAngle);
                //bot.driveXYW(0,0,0);
                timer = new ElapsedTime();
            }
            @Override
            void update(){
                // Updating pid to move back to rest position if pushed out of it
                //pid.update();
                bot.driveXYW(0,0,0);

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
    
    // public void findSample(){
    //     steps.add(new BotState("FINDING SAMPLE TO INTAKE..."){
    //         ElapsedTime timer;
    //         ElapsedTime wiggleTimer;
    //         AutoBotLL autobot = (AutoBotLL)bot;
    //         double maxPower = .4;
    //         double minPower = .2;
    //         boolean wiggleLeft = false;
            
    //         @Override void start(){
    //             timer = new ElapsedTime();
    //             wiggleTimer = new ElapsedTime();
    //         }
    //         @Override
    //         void update(){
    //             LLResult result = autobot.limelight.getLatestResult();
    //             if(result != null){
    //                 DetectorResult target = result.getDetectorResults().get(0);
    //                 double xPower = 0;
    //                 double yPower = 0;
                    
    //                 if(target!=null){
    //                     double targetX = target.getTargetXDegrees();
    //                     xPower = (double)(Math.abs(targetX)-minPower)/(maxPower-minPower);
    //                     double targetY = target.getTargetYDegrees();
    //                     yPower = (double)(Math.abs(targetY)-minPower)/(maxPower-minPower);
                        
    //                     xPower*=Math.signum(targetX);
    //                     yPower*=Math.signum(targetY);
    //                     autobot.driveXYW(xPower,yPower,0);
    //                 }else{
    //                     autobot.driveXYW(0,0,wiggleLeft?-.3:.3);
    //                 }
                    
    //                 if(xPower<.2 && yPower<.2){
    //                     nextState();
    //                 }
    //             }
                
    //             if(wiggleTimer.seconds()>.35){
    //                 wiggleTimer.reset();
    //                 wiggleLeft = !wiggleLeft;
    //             }
                
    //             if(timer.seconds()>1.5){
    //                 nextState();
    //             }
    //         }
    //     });
    // }
}