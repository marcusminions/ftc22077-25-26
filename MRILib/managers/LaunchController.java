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
        FIRE,  // Aim and fire
        POWER,  // Power flywheels, but don't aim at target
        AIM  // Aim at target with power, but do not fire
    }

    // Volatile stuff can get updated from outside
    public volatile boolean running = true;
    public volatile LaunchMode launchMode = LaunchMode.OFF;
    public volatile Pose2D target;
    public volatile double launchAngle = 30d;

    private volatile double leftSpinVel;
    private volatile double rightSpinVel;

    private volatile Pose2D position;
    private volatile Pose2D velocity;

    private double leftSpinVelPrev;
    private double rightSpinVelPrev;
    
    private double deltaTheta;
    private double targetLeftSpinVel;
    private double targetRightSpinVel;

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
            // Init variables?
            
            // Time control to maintain thread-safety
            double currentTime = timer.seconds();
            double deltaTime = currentTime-previousTime;
            previousTime = currentTime;

            // VENKAT DO THE FANCY MATH HERE
            // ...
            // ...

            // Placeholders
            deltaTheta = 0.0;
            targetLeftSpinVel = 0.0;
            targetRightSpinVel = 0.0;

            // Now, only launch if on
            switch (launchMode) {
            case FIRE:
                // Power firing using above calculated firing solution.
                overrideSteering(true, deltaTheta);
                
                bot.setLeftSpinVelocity(targetLeftSpinVel);
                bot.setRightSpinVelocity(targetRightSpinVel);

            case POWER:
                // Hold power, but do not aim at target
                overrideSteering(false, 0d);

                bot.setLeftSpinVelocity(targetLeftSpinVel);
                bot.setRightSpinVelocity(targetRightSpinVel);

            case AIM:
                // Aim towards target, hold flywheel power, but do not fire
                overrideSteering(true, deltaTheta);

                bot.setLeftSpinVelocity(targetLeftSpinVel);
                bot.setRightSpinVelocity(targetRightSpinVel);

            case OFF:
                // Nothing happens
                overrideSteering(false, 0d);
            }
            
            // END OF LOOP ACTIONS -- Waiting until next loop
            while(System.nanoTime() < nextLoopTime){}
            nextLoopTime += loopTime * 1_000_000;
        }
    }

    public synchronized void updateCurrentState(Pose2D position, Pose2D velocity, double leftSpinV, double rightSpinV) {
        this.position = position;
        this.velocity = velocity;
        leftSpinVel = leftSpinV;
        rightSpinVel = rightSpinV;
    }

    public synchronized void setLaunchMode(LaunchMode mode) {
        launchMode = mode;
    }

    public synchronized void setLaunchTarget(Pose2D t) {
        target = t;
    }

    public void stop() {
        running = false;
    }
}