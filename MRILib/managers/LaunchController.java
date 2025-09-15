package MRILib.managers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


// IS multithreaded
public class LaunchController implements Runnable {

    private LaunchBot bot;
    private Telemetry telemetry;

    public enum LaunchMode {
        OFF,  // No motor action
        FIRE, // Aim and fire
        HOLD  // Aim at target, but do not fire
    }

    // Volatile stuff can get updated from outside
    public volatile LaunchMode launchMode = LaunchMode.OFF;
    public volatile Pose2D launchTarget;
    public volatile boolean running = true;

    private volatile double leftSpinVel;
    private volatile double rightSpinVel;

    private volatile Pose2D position;
    private volatile Pose2D velocity;

    private double leftSpinVelPrev;
    private double rightSpinVelPrev;
    
    public LaunchController(LaunchBot bot) {
        this.bot = bot;
        telemetry = bot.getTelemetry();
    }
    
    @Override
    public void run() {
        long loopTime = 10;
        long nextLoopTime = System.nanoTime() + loopTime * 1_000_000;
        ElapsedTime timer = new ElapsedTime();
        double previousTime = timer.seconds();
        
        while (running) {
            // Init variables
            
            // Time control to maintain thread-safety
            double currentTime = timer.seconds();
            double deltaTime = currentTime-previousTime;
            previousTime = currentTime;

            // DO FANCY MATH HERE
            // ...
            // ...
            
            // Theoretically, always have aiming system (if mechanically present) facing towards goal, therefore;
            // If facing direction, turn launcher towards proper firing angle

            // Now, only launch if on
            switch (launchMode) {
                case FIRE:
                    // Power firing using above calculated firing solution.
                case HOLD:
                    // Aim towards target, but do not fire
                case OFF:
                    // Nothing happens
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

    public synchronized void setLaunchTarget(Pose2D target) {
        launchTarget = target;
    }

    public void stop() {
        running = false;
    }
}