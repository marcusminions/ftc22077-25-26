package MRILib.managers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static MRILib.BotValues.*;

public class LaunchBot extends Bot {

    private DcMotorEx left;
    private DcMotorEx right;
    private DcMotor conveyor;
    private DcMotor intake;
    private Servo kicker;

    private LaunchController launchController;
    private Thread launchThread;

    public int leftPos;
    public int rightPos;
    public int leftPosPrev;
    public int rightPosPrev;
    
    public LaunchBot (LinearOpMode op) {
        super(op);
        initServos();
        initMotors();
        launchController = new LaunchController(this);
        launchThread = new Thread(launchController);
        resetEncoders();
    }

    // Update launch controller
    public void update() {
        super.update();
        launchController.updateCurrentState(currentPosition, velocity, getLeftVelocity(), getRightVelocity());
        updateEncoders();
    }

    // Initialize all motors not part of drivetrain
    public void initMotors() {
        left = op.hardwareMap.get(DcMotorEx.class, "shootLeft");
        right = op.hardwareMap.get(DcMotorEx.class, "shootRight");
        conveyor = op.hardwareMap.get(DcMotor.class, "conveyor");
        intake = op.hardwareMap.get(DcMotor.class, "intake");
        
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        left.setDirection(MLDIR);
        right.setDirection(MRDIR);
        conveyor.setDirection(CONVEYDIR);
        intake.setDirection(INDIR);
        
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetEncoders();
    }

    // Initialize all servos
    public void initServos() {
        kicker = op.hardwareMap.get(Servo.class, "kicker");
    }

    // Update current and last encoder positions
    public void updateEncoders() {
        super.updateEncoders();
        leftPosPrev = leftPos;
        rightPosPrev = rightPos;

        leftPos = left.getCurrentPosition();
        rightPos = right.getCurrentPosition();
    }

    // Resetting encoders to 0, keeping current mode
    public void resetEncoders() {
        super.resetEncoders(); //calling Bot.resetEncoders()
        DcMotorEx.RunMode leftMode = left.getMode();
        DcMotorEx.RunMode rightMode = right.getMode();

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(leftMode);
        right.setMode(rightMode);
    }

    // Getters
    public int getLeftPos()          { return leftPos; }
    public int getRightPos()         { return rightPos; }
    public double getLeftVelocity()  { return left.getVelocity(); }
    public double getRightVelocity() { return right.getVelocity(); }

    // Setters
    public void setLeftPower(double power)        { left.setPower(power); }
    public void setRightPower(double power)       { right.setPower(power); }
    public void setLeftVelocity(double velocity)  { left.setVelocity(velocity); }
    public void setRightVelocity(double velocity) { right.setVelocity(velocity); }

    public void setConveyorPower(double power)        { conveyor.setPower(power); }
    public void setIntakePower(double power)          { intake.setPower(power); }
    public void setKickerPosition(double position)    { kicker.setPosition(position); }

    public void setIntakeDirection(DcMotor.Direction dir) { intake.setDirection(dir); }
    

    public void setLaunchControllerTarget(Pose2D target) {
        launchController.setLaunchTarget(target);
    }
    
    public void setLaunchControllerAimMode(LaunchController.LaunchMode mode) {
        launchController.setAimMode(mode);
    }
    
    public void setLaunchControllerPowerMode(LaunchController.LaunchMode mode) {
        launchController.setPowerMode(mode);
    }
    
    public boolean launchReady() { return launchController.launchReady(); }

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