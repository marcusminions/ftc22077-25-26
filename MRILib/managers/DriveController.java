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


// Multithread driving, hopefully improves performance
public class DriveController implements Runnable {
    
    private Bot bot;
    private Telemetry telemetry;

    // Volatile stuff is subject to change from other threads
    // constantOrdering = true means motors get same order every time
    public volatile boolean smartOrdering = false;

    public volatile boolean running = true;

    public volatile double flPower;
    public volatile double frPower;
    public volatile double blPower;
    public volatile double brPower;

    private volatile double lastFLPower;
    private volatile double lastFRPower;
    private volatile double lastBLPower;
    private volatile double lastBRPower;

    public volatile DcMotorEx frontLeft;
    public volatile DcMotorEx frontRight;
    public volatile DcMotorEx backLeft;
    public volatile DcMotorEx backRight;

    public DriveController(Bot bot) {
        this.bot = bot;
        telemetry = bot.getTelemetry();

        frontLeft = bot.getFL();
        frontRight = bot.getBR();
        backLeft = bot.getBL();
        backRight = bot.getBR();
    }

    @Override
    public void run() {
        // No fancy looping here
        
        while (running) {
            // Find out which ways the robot is trying to go
            double forward = flPower + frPower + blPower + brPower;
            double right =   flPower - frPower - blPower + brPower;
            double turnCW =  flPower - frPower + blPower - brPower;

            // Find primary direction of travel, always use front wheel drive
            // Forward
            if (Math.abs(forward) >= Math.abs(right) &&
                Math.abs(forward) >= Math.abs(turnCW) &&
                forward >= 0d) {
                    frontLeft.setPower(flPower);
                    frontRight.setPower(frPower);
                    backRight.setPower(brPower);
                    backLeft.setPower(blPower);
            // Backward
            } else if (Math.abs(forward) >= Math.abs(right) &&
                Math.abs(forward) >= Math.abs(turnCW) &&
                forward <= 0d) {
                    backLeft.setPower(blPower);
                    backRight.setPower(brPower);
                    frontRight.setPower(frPower);
                    frontLeft.setPower(flPower);
            // Right
            } else if (Math.abs(right) >= Math.abs(forward) &&
                Math.abs(right) >= Math.abs(turnCW) &&
                right >= 0d) {
                    backRight.setPower(brPower);
                    frontRight.setPower(frPower);
                    frontLeft.setPower(flPower);
                    backLeft.setPower(blPower);
            // Left
            } else if (Math.abs(right) >= Math.abs(forward) &&
                Math.abs(right) >= Math.abs(turnCW) &&
                right <= 0d) {
                    backLeft.setPower(blPower);
                    frontLeft.setPower(flPower);
                    frontRight.setPower(frPower);
                    backRight.setPower(brPower);
            // Rotation (not a big deal)
            } else {
                frontLeft.setPower(flPower);
                backRight.setPower(blPower);
                frontRight.setPower(frPower);
                backLeft.setPower(blPower);
            } 

        }
    }

    public void setPowers(double fl, double fr, double bl, double br) {
        lastFLPower = flPower;
        lastFRPower = frPower;
        lastBLPower = blPower;
        lastBRPower = brPower;

        flPower = fl;
        frPower = fr;
        blPower = bl;
        brPower = br;
    }

    public void stop() {
        running = false;
    }
}