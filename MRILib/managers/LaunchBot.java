package MRILib.managers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static MRILib.BotValues.*;

public class LaunchBot extends Bot {

    private DcMotorEx leftSpin;
    private DcMotorEx rightSpin;

    private LaunchController launchController;
    private Thread launchThread;

    public int leftSpinPos;
    public int rightSpinPos;
    public int leftSpinPosPrev;
    public int rightSpinPosPrev;
    
    public LaunchBot (LinearOpMode op) {
        super(op);
        initMotors();
        launchController = new LaunchController(this);
        launchThread = new Thread(launchController);
        resetEncoders();
    }

    // Update launch controller
    public void update() {
        super.update();
        launchController.updateCurrentState(currentPosition, velocity, getleftSpinVelocity(), getrightSpinVelocity());
        updateEncoders();
    }

    // Initialize all motors not part of drivetrain
    public void initMotors() {
        leftSpin = op.hardwareMap.get(DcMotorEx.class, "leftSpin");
        rightSpin = op.hardwareMap.get(DcMotorEx.class, "rightSpin");
        
        leftSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftSpin.setDirection(MLDIR);
        rightSpin.setDirection(MRDIR);
        
        leftSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    // Update current and last encoder positions
    public void updateEncoders() {
        super.updateEncoders();
        leftSpinPosPrev = leftSpinPos;
        rightSpinPosPrev = rightSpinPos;

        leftSpinPos = leftSpin.getCurrentPosition();
        rightSpinPos = rightSpin.getCurrentPosition();
    }

    // Resetting encoders to 0, keeping current mode
    public void resetEncoders() {
        super.resetEncoders(); //calling Bot.resetEncoders()
        DcMotorEx.RunMode leftSpinMode = leftSpin.getMode();
        DcMotorEx.RunMode rightSpinMode = rightSpin.getMode();

        leftSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSpin.setMode(leftSpinMode);
        rightSpin.setMode(rightSpinMode);
    }

    // Getters
    public int getleftSpinPos()         { return leftSpinPos; }
    public int getrightSpinPos()         { return rightSpinPos; }
    public double getleftSpinVelocity() { return leftSpin.getVelocity(); }
    public double getrightSpinVelocity() { return rightSpin.getVelocity(); }

    // Setters
    public void setleftSpinPower(double power)       { leftSpin.setPower(power); }
    public void setrightSpinPower(double power)       { rightSpin.setPower(power); }
    public void setleftSpinVelocity(double velocity) { leftSpin.setVelocity(velocity); }
    public void setrightSpinVelocity(double velocity) { rightSpin.setVelocity(velocity); }

    public void setLaunchControllerMode(LaunchController.LaunchMode mode) {
        launchController.setLaunchMode(mode);
    }

    public void setLaunchControllerTarget(Pose2D target) {
        launchController.setLaunchTarget(target);
    }

    // Deal with threads
    public void startMultiThread() { launchThread.start(); }
    public void stopMultiThread() {
        launchController.stop();
        try {
            launchThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}