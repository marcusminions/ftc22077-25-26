package MRILib.managers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.pmrobotics.ledmatrix.GoBildaPinpointDriver;

//import MRILib.util.GoBildaPinpointDriver;
import static MRILib.BotValues.*;

/*
 * This file contains the initialization of the drive train motors, imu, 
 * odometry computer, and voltage sensor
 * as well as all getter, setter, and helper methods for any related values
 * 
 * It is a superclass for ArmBot, which implements the same helper methods but for
 * any arm and game unique motors, servos, and sensors
 * 
 * While the ArmBot class and its subclasses need to be different for every individual robot,
 * this class should not change besides a few initialization parameters
 * 
 * This file is specifically illustrates the manager and initialization class for a
 * 4-motor Omni-Directional (or Holonomic) Drive-train using GoBilda Mechanum wheels and the
 * GoBilda Pinpoint Odometry Computer I2C device. It can be easily modified, however, to instead utilize a 
 * custom odometry class using wheel encoders or a 3 dead-wheel odometry system by switching out the odo
 * variable and making sure all the method names line up between the calls on this file and the 
 * names on your odometry file.
 */

public class Bot {

    //creating motor variables to be initialized in initMotors()
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    
    //creating odometry computer to be initialized in initOdo()
    public GoBildaPinpointDriver odo;
    
    double angleOffset = 0d;

    //creating on-board IMU to be initialized in initIMU()
    public IMU imu;
    private boolean usingIMU;
    private boolean usingDriveController;

    //creating voltage sensor object
    private VoltageSensor voltageSensor;

    private DriveController driveController;
    private Thread driveThread;

    // These are volatile because used by LaunchController
    //current and last position, calculated by the odometry computer
    public volatile Pose2D currentPosition;
    public volatile Pose2D lastPosition;

    //current velocity calculated from current and last positions
    public volatile Pose2D velocity;
    
    protected LinearOpMode op;
    
    //current motor positions
    int frontLeftPos  = 0;
    int frontRightPos = 0;
    int backLeftPos   = 0;
    int backRightPos  = 0;

    //previous motor positions
    int frontLeftPosPrev  = 0;
    int frontRightPosPrev = 0;
    int backLeftPosPrev   = 0;
    int backRightPosPrev  = 0;

    public Bot(LinearOpMode op)
    { //constructor
        this.op = op;
        initMotors();
        initIMU();
        initOdo();
        driveController = new DriveController(this);
        driveThread = new Thread(driveController);
        voltageSensor = op.hardwareMap.voltageSensor.iterator().next();
    }

    private void initMotors()
    { //initializing drive motors
        frontLeft = op.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = op.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = op.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = op.hardwareMap.get(DcMotorEx.class, "backRight");

        //setting the direction of the motors so all go forward when set to positive power
        setDirections();
    }

    public void initIMU()
    { //initializing on board IMU

        //initializing the imu object using the hardwareMap from your OpMode
        imu = op.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));

        //initializing the imu with the specified orientation parameters
        imu.initialize(params);
    }

    public void initOdo()
    { //initialize odometry

        //initializing the odometry object using the hardwareMap from your OpMode
        odo = op.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // setting the init parameters of the two odometry wheels 
        /*
            USE MILIMETERS
            Set the odometry pod positions relative to the point that the odometry computer tracks around.
            The X pod offset refers to how far sideways from the tracking point the
            X (forward) odometry pod is. Left of the center is a positive number,
            right of center is a negative number. the Y pod offset refers to how far forwards from
            the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
            backwards is a negative number.
        */
        odo.setOffsets(26.0, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // resetting the position and heading to zero (this can be reset to a different starting position
        // using setPosition())
        odo.resetPosAndIMU();
    }

    public void update(){
        //update robot position and encoder ticks in each opMode loop tick
        odo.update();
        lastPosition = currentPosition;
        currentPosition = getOdoPosition();
        velocity = getOdoVelocity();
        updateEncoders();
    }

    public void updateEncoders()
    { //updating current position numbers

        //saving the current motor positions as the position in the last tick
        frontLeftPosPrev = frontLeftPos;
        frontRightPosPrev = frontRightPos;
        backLeftPosPrev = backLeftPos;
        backRightPosPrev = backRightPos;

        //updating the current motor positions to their new values
        frontLeftPos = frontLeft.getCurrentPosition();
        frontRightPos = frontRight.getCurrentPosition();
        backLeftPos = backLeft.getCurrentPosition();
        backRightPos = backRight.getCurrentPosition();
    }

    

    //getter methods
    public DcMotorEx getFL(){ return frontLeft;     }
    public DcMotorEx getFR(){ return frontRight;    }
    public DcMotorEx getBL(){ return backLeft;      }
    public DcMotorEx getBR(){ return backRight;     }
    public int getFLPos(){ return frontLeftPos;  }
    public int getFRPos(){ return frontRightPos; }
    public int getBLPos(){ return backLeftPos;   }
    public int getBRPos(){ return backRightPos;  }
    public int getFLPosPrev() { return frontLeftPosPrev;  }
    public int getFRPosPrev() { return frontRightPosPrev; }
    public int getBLPosPrev() { return backLeftPosPrev;   }
    public int getBRPosPrev() { return backRightPosPrev;  }

    public double getIMUHeading(){ return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    public void resetIMUHeading(){ imu.resetYaw(); }
    
    public Pose2D getPosition(){
        // returning the current odometry position output by the odometry computer
        return currentPosition;
    }
    public Pose2D getLastPosition(){
        // returning the odometry position as it was at the last update tick
        return lastPosition;
    }

    public Pose2D getVelocity(){
        // return last velocity
        return velocity;
    }

    private Pose2D getOdoPosition(){
        // returns the odo position direct from the odometry computer
        // (this is private so that getPosition is used, which is properly synced with the update ticks)
        Pose2D odoPose = odo.getPosition();
        Pose2D corrected = new Pose2D(DistanceUnit.INCH, odoPose.getY(DistanceUnit.INCH), odoPose.getX(DistanceUnit.INCH), AngleUnit.DEGREES, 0);
        return corrected;
    }
    private Pose2D getOdoVelocity(){
        // Returns the velocity according to the pinpoint computer
        double x = odo.getVelX(DistanceUnit.INCH);
        double y = odo.getVelY(DistanceUnit.INCH);
        Pose2D velocity = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);
        return velocity;
    }
    public double getX(){
        // returns the X value of the current odometry position
        return getOdoPosition().getX(DistanceUnit.INCH);
    }
    public double getY(){
        // returns the Y value of the current odometry position
        return getOdoPosition().getY(DistanceUnit.INCH);
    }
    public double getHeading(){
        // returns the heading value of the current odometry position using 
        // the odometry computer's IMU by default (it seems to be more consistent than the on-board IMU)
        double a = getIMUHeading() - angleOffset;
        if (a > 180) a -= 360;
        if (a < -180) a += 360;
        return a;
    }
    
    public void resetHeading() {
        angleOffset = 0;
        angleOffset = getHeading() + 180;
    }

    public synchronized double getVoltage()
    { // returning the current output voltage from the battery to the control hub
        return voltageSensor.getVoltage();
    }
    
    public Telemetry getTelemetry(){
        return op.telemetry;
    }
    
    public void setHeading(double heading){
        setPosition(new Pose2D(DistanceUnit.INCH, getX(), getY(), AngleUnit.DEGREES, heading));
        angleOffset = -heading;
    }
    
    public void setPosition(double x, double y){
        setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, odo.getPosition().getHeading(AngleUnit.DEGREES)));
    }
    public void setPosition(double x, double y, double heading){
        //overloading setPosition to take an x, y, and heading instead of a pose for QOL
        setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
        angleOffset = -heading;
    }
    public void setPosition(Pose2D pos)
    { // overriding the current position read by the Gobilda Pinpoint Odometry Computer
        Pose2D corrected = new Pose2D(DistanceUnit.INCH, pos.getY(DistanceUnit.INCH), pos.getX(DistanceUnit.INCH), AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES));
        odo.setPosition(corrected);
        angleOffset = -pos.getHeading(AngleUnit.DEGREES);
    }

    public void setDirections()
    { //setting the directions of the motors to their values specified in the static BotValues
        frontLeft.setDirection(RIGHTDIR);
        frontRight.setDirection(LEFTDIR);
        backLeft.setDirection(RIGHTDIR);
        backRight.setDirection(LEFTDIR);
    }
    
    public void setMode(DcMotor.RunMode mode)
    { //setting motors to an input runmode
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void resetEncoders()
    { //Resetting encoders to 0, keeping current mode

        //setting the previous encoder tick values to zero to reflect the new encoder reset
        resetPrevEncoders();

        //saving the runmode of the drive motors so that it can be returned to after the motors are reset
        DcMotor.RunMode mode = frontLeft.getMode();

        //stop and resetting the encoder values of each drive motor
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reapplying the saved runmode
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void resetPrevEncoders(){
        frontLeftPosPrev = 0;
        frontRightPosPrev = 0;
        backLeftPosPrev = 0;
        backRightPosPrev = 0;
    }

    public void enableBrakeMode(boolean enabled)
    { // turning on or off brakemode
        DcMotor.ZeroPowerBehavior state = enabled ? BRAKE : FLOAT;
        frontLeft.setZeroPowerBehavior(state);
        frontRight.setZeroPowerBehavior(state);
        backLeft.setZeroPowerBehavior(state);
        backRight.setZeroPowerBehavior(state);
    }
    
    public void toggleUsingIMU(){
        usingIMU = !usingIMU;
    }
    public boolean getUsingIMU(){
        return usingIMU;
    }
    public void useIMU(){usingIMU=true;}
    public void useOdo(){usingIMU=false;}

    public void driveXYW(double rx, double ry, double rw)
    { // sets proportional power to drive motors

        //    The input x, y, and w represent a 2 dimensional vector for the intended
        //    direction of motion, as well as an intended power for rotation around 
        //    the z axis (spinning in a circle)
        //    
        //    this is typically input using the gamepad with -left_stick_x being jy, -left_stick_y being jx, and
        //    right_stick_x being jw
        //    
        //    calculates the power that needs to go to each of the four mechanum wheels
        //    in order to push the robot in the direction specified by the 2D vector, while applying further power
        //    to rotate the target amount
        //    
        //    these power values are then proportionally clamped to stay within a maximum power value of 1, as well
        //    as regulating the max speed using a voltage sensor multiplier
        

        // denom is the multiplier applied to the power calculations in order to 
        // ensure the motors all remain <= 1 but are still proportionally the same
        double denom = Math.max(Math.abs(rx)+Math.abs(ry)+Math.abs(rw),1);

        // dividing the power by a multiplier to counteract variance in motor voltage above 12 volts
        // ensuring that power doesnt get unexpectedly high, causing precise movements to get inaccurate 
        double voltageMulti = getVoltage() / 12;

        // adding and subtracting the x, y, and theta power values for each wheel to
        // push the robot in the vector direction made when combining all three powers
        double lfPower = ((rx - ry - rw) / denom) / voltageMulti;
        double rfPower = ((rx + ry + rw) / denom) / voltageMulti;
        double lbPower = ((rx + ry - rw) / denom) / voltageMulti;
        double rbPower = ((rx - ry + rw) / denom) / voltageMulti;
        
        // applying calculated vector powers to each motor
        if (usingDriveController) {
            driveController.setPowers(lfPower, rfPower, lbPower, rbPower);
        } else {
            frontLeft.setPower(lfPower);
            frontRight.setPower(rfPower);
            backLeft.setPower(lbPower);
            backRight.setPower(rbPower);
        }
        
        // TESTING ONLY
        // backRight.setPower(.8);
        // frontLeft.setPower(.8);
        // frontRight.setPower(.8);
        // backLeft.setPower(.8);
    }
    
    public void driveFieldXYW(double fx, double fy, double fw)
    { // rotate field orientation inputs to robot orientation

        //   takes the same inputs as driveXYW but rotates the power values for
        //   field centric driver control
        //
        //   field centric driving is a driver control mode which results in the inputs of the driver, i.e. x, y, and w
        //   pushing the robot relative to the field and driver instead of itself
        //   effectively causing movement to be the same regardless of the robot's current orientation in respect to yaw
        //
        //   although initially confusing and disorientating, this driver control mode offers beneficial maneuverability
        //   and at the end of the day makes more sense for the maneuvering that FTC robots must perform in their matches


        // getting the current robot's yaw, and converting it to a usable form to calculate the offset
        double theta = Math.toRadians(-getHeading());
        if(usingIMU){
            theta = Math.toRadians(-getIMUHeading());
        }
        //double theta = Math.toRadians(-getHeading());

        //calculating the rotated x power
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        //calculating the rotated y power
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        // inputting the new rotated x and y vector to driveXYW
        // passing through the rotation value (fw) as rotation around the robot's z axis
        // is unaffected by rotation of the directional power vector
        driveXYW(rx, ry*1.15, fw);
    }

    // Deal with threading
    public void enableDriveThread() {
        usingDriveController = true;
        startDriveThread();
    }
    public void disableDriveThread() {
        usingDriveController = false;
        stopDriveThread();
    }

    public void startDriveThread() { driveThread.start(); }
    public void stopDriveThread() {
        driveController.stop();
        try {
            driveThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
