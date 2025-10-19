package MRILib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GameValues {

    // Miscellaneous variables
    public static enum COLOR {
        RED,
        BLUE,
        GREEN,
        PURPLE
    }

    /* --- FIELD VALUES (inches) ---
     * Base Zone:          18   x 18
     * Gate Zone:          2.75 x 10
     * Loading Zone:       23   x 23
     * Secret Tunnel Zone: 46.5 x 6.125
     * Spike Mark:        ~1    x 10
     */

    // Field positions, directions are relative to alliance
    public static Pose2D RED_GOAL =         new Pose2D(DistanceUnit.INCH, -65, 65, AngleUnit.DEGREES, 0);
    public static Pose2D RED_SPIKE_LEFT =   new Pose2D(DistanceUnit.INCH, -12, -48, AngleUnit.DEGREES, 0);
    public static Pose2D RED_SPIKE_MIDDLE = new Pose2D(DistanceUnit.INCH, 12, -48, AngleUnit.DEGREES, 0);
    public static Pose2D RED_SPIKE_RIGHT =  new Pose2D(DistanceUnit.INCH, 36, -48, AngleUnit.DEGREES, 0);

    public static Pose2D BLUE_GOAL =         new Pose2D(DistanceUnit.INCH, -65, -65, AngleUnit.DEGREES, 0);
    public static Pose2D BLUE_SPIKE_LEFT =   new Pose2D(DistanceUnit.INCH, 36, 48, AngleUnit.DEGREES, 0);
    public static Pose2D BLUE_SPIKE_MIDDLE = new Pose2D(DistanceUnit.INCH, 12, 48, AngleUnit.DEGREES, 0);
    public static Pose2D BLUE_SPIKE_RIGHT =  new Pose2D(DistanceUnit.INCH, -12, 48, AngleUnit.DEGREES, 0);

    // Robot Constants
    public static final double WIDTH = 12;
    public static final double LENGTH = 15d;
    public static final int INVENTORY_SIZE = 2;

    // Launch related constants
    public static double KICK = 1;  // Kicker servo positions
    public static double BACK = 0;
    public static double KICK_TIME = 2;
    public static int DEFAULT_LAUNCH_MODIFIDER = 0;
    public static double RAMP_POWER = 1;
    public static double INTAKE_POWER = .7;

    // Hopefully linear value that will translate velocity of motors using motor.setVelocity()
    // to release velocity of the ball in meters per second.
    public static final double TICK_TO_RELEASE_VELOCITY = 0;
    public static final double GRAVITY = 9.807;  // m/s^2
    public static final double GOAL_HEIGHT = .9845;  // We want to aim for 7cm above this

    /* --- FORMULAS ---
     *
     * FOR STATIONARY FIRING
     *          __________________
     * v0 =  d / ________g_______
     *       \/ 2cos2(0)(dtan0 - y)
     * v0 = d * sqrt[g / 2cos^2(0)(dtan0 - y)]
     * 
     * v0: initial launch velocity
     * g: gravity, 9.81 m/s
     * d: distance (launcher to middle of goal)           /
     * y: change in height (launcher to top of goal)     /  0
     * 0: launch angle relative to horizontal           /)____
     */
}