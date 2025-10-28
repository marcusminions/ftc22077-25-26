package MRILib.managers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import MRILib.eventlistener.BotEventManager;
import MRILib.eventlistener.BotEventManager.*;
import MRILib.eventlistener.BotEvent;
import MRILib.GameValues.*;


// IS multithreaded
public class LaunchController implements Runnable {

    private LaunchBot bot;
    private Telemetry telemetry;

    public enum LaunchMode {
        OFF,  // No motor action
        POWER,  // Power flywheels
        AIM  // Aim at target
    }

    // Volatile stuff can get updated from outside
    public volatile boolean running = true;
    private volatile LaunchMode aimMode = LaunchMode.OFF;
    private volatile LaunchMode powerMode = LaunchMode.OFF;
    public volatile Pose2D target;
    public volatile double launchAngle = 30d;

    private volatile double leftVel;
    private volatile double rightVel;

    private volatile Pose2D position;
    private volatile Pose2D velocity;

    private double leftVelPrev;
    private double rightVelPrev;
    
    private double deltaTheta;
    private double targetLeftVel;
    private double targetRightVel;

    public LaunchController(LaunchBot bot) {
        this.bot = bot;
        telemetry = bot.getTelemetry();
    }

    private void overrideSteering(boolean o, double a) {
        BotEventManager.broadcast(EventType.OVERRIDE_DIRECTION, new BotEvent(){ 
            public boolean override = o;
            public double angle = a;
        });
    }
    
    @Override
    public void run() {
        long loopTime = 10;
        long nextLoopTime = System.nanoTime() + loopTime * 1_000_000;
        ElapsedTime timer = new ElapsedTime();
        double previousTime = timer.seconds();

        while (running) {
            // Time control to maintain thread-safety
            double currentTime = timer.seconds();
            double deltaTime = currentTime-previousTime;
            previousTime = currentTime;

            // VENKAT DO THE FANCY MATH HERE
            // ...
            // ...

            // Placeholders
            deltaTheta = 0.0;
            targetLeftVel = 2000.0;
            targetRightVel = 2000.0;

            // Now, only launch if on
            if (aimMode == LaunchMode.AIM) overrideSteering(true, deltaTheta);
            if (powerMode == LaunchMode.POWER) {
                bot.setLeftVelocity(targetLeftVel);
                bot.setRightVelocity(targetRightVel);
            } else {
                bot.setLeftPower(0);
                bot.setRightPower(0);
            }
            
            // END OF LOOP ACTIONS -- Waiting until next loop
            while(System.nanoTime() < nextLoopTime){}
            nextLoopTime += loopTime * 1_000_000;
        }
    }

    public void updateCurrentState(Pose2D position, Pose2D velocity, double leftV, double rightV) {
        this.position = position;
        this.velocity = velocity;
        leftVel = leftV;
        rightVel = rightV;
    }
    
    public boolean launchReady() {
        if (Math.abs(bot.getLeftVelocity()) > Math.abs(targetLeftVel) - 20 &&
            Math.abs(bot.getRightVelocity()) > Math.abs(targetRightVel) - 20) {
            return true;
        } else return false;
    }

    public void setAimMode(LaunchMode mode) { aimMode = mode; }
    public void setPowerMode(LaunchMode mode) { powerMode = mode; }
    public void setLaunchTarget(Pose2D t) { target = t; }

    public void stop() {
        running = false;
    }
}