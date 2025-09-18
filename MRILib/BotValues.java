package MRILib;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

public class BotValues {
    public static double armOffset = -14; //-19
    public static double teleOpArmOffset = 0;
    
    public static DcMotor.Direction LEFTDIR = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction RIGHTDIR = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction SLIDEDIR = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction PIVOTDIR = DcMotor.Direction.REVERSE;
    public static LogoFacingDirection LOGO_DIR = LogoFacingDirection.LEFT;
    public static UsbFacingDirection USB_DIR = UsbFacingDirection.UP;

    public static DcMotor.Direction MLDIR = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction MRDIR = DcMotor.Direction.FORWARD;

    // define robot-specific motor/wheel constants
    public static final double TICKS_PER_REVOLUTION = 384.5; //384.5
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_DIAMETER = 4.09; //3.78

    // define calculated constants
    public static final double WHEEL_CIRCUMFERENCE =
          WHEEL_DIAMETER * Math.PI;
    public static final double DISTANCE_PER_REVOLUTION =
          WHEEL_CIRCUMFERENCE * GEAR_RATIO * 1.7;

    public static final DcMotor.ZeroPowerBehavior BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final DcMotor.ZeroPowerBehavior FLOAT = DcMotor.ZeroPowerBehavior.FLOAT;

    public static final double TICKS_PER_REVOLUTION_ARM = 751.8;
    public static final double GEAR_RATIO_ARM = 20.0/74.0;
    public static final double TICKS_PER_DEGREE_ARM = TICKS_PER_REVOLUTION_ARM/(GEAR_RATIO_ARM*360.0);
    
    public static final double TICKS_PER_REVOLUTION_SLIDE = 103.8;
    //public static final double DISTANCE_PER_REVOLUTION_SLIDE = TICKS_PER_REVOLUTION_SLIDE/(GEAR_RATIO_SLIDE*360);
    public static final double WHEEL_DIAMETER_SLIDE = .150; //in meters
    public static final double WHEEL_CIRCUMFERENCE_SLIDE = 128;
    public static final double DISTANCE_PER_TICK_SLIDE = WHEEL_CIRCUMFERENCE_SLIDE/TICKS_PER_REVOLUTION_SLIDE;
}