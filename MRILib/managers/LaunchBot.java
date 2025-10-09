package MRILib.managers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static MRILib.BotValues.*;

public class LaunchBot extends Bot {

    private DcMotorEx leftSpin;
    private DcMotorEx rightSpin;
    private DcMotor conveyor;
    private DcMotor intake;
    private Servo kicker;

    private LaunchController launchController;
    private Thread launchThread;

    public int leftSpinPos;
    public int rightSpinPos;
    public int leftSpinPosPrev;
    public int rightSpinPosPrev;
    
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
        launchController.updateCurrentState(currentPosition, velocity, getLeftSpinVelocity(), getRightSpinVelocity());
        updateEncoders();
    }

    // Initialize all motors not part of drivetrain
    public void initMotors() {
        leftSpin = op.hardwareMap.get(DcMotorEx.class, "shootLeft");
        rightSpin = op.hardwareMap.get(DcMotorEx.class, "shootRight");
        conveyor = op.hardwareMap.get(DcMotor.class, "conveyor");
        intake = op.hardwareMap.get(DcMotor.class, "intake");
        
        leftSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        leftSpin.setDirection(MLDIR);
        rightSpin.setDirection(MRDIR);
        conveyor.setDirection(CONVEYDIR);
        intake.setDirection(INDIR);
        
        leftSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
    public int getLeftSpinPos()          { return leftSpinPos; }
    public int getRightSpinPos()         { return rightSpinPos; }
    public double getLeftSpinVelocity()  { return leftSpin.getVelocity(); }
    public double getRightSpinVelocity() { return rightSpin.getVelocity(); }

    // Setters
    public void setLeftSpinPower(double power)        { leftSpin.setPower(power); }
    public void setRightSpinPower(double power)       { rightSpin.setPower(power); }
    public void setLeftSpinVelocity(double velocity)  { leftSpin.setVelocity(velocity); }
    public void setRightSpinVelocity(double velocity) { rightSpin.setVelocity(velocity); }

    public void setConveyorPower(double power)        { conveyor.setPower(power); }
    public void setIntakePower(double power)          { intake.setPower(power); }
    public void setKickerPosition(double position)    { kicker.setPosition(position); }

    public void setIntakeDirection(DcMotor.Direction dir) { intake.setDirection(dir); }

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