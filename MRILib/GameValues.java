package MRILib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GameValues {

    // Field positions
    public static Pose2D RED_TARGET = new Pose2D(DistanceUnit.INCH, -65, 65, AngleUnit.DEGREES, 0);
    public static Pose2D BLUE_TARGET = new Pose2D(DistanceUnit.INCH, -65, -65, AngleUnit.DEGREES, 0);

    // Robot Constants
    public static final double SMALLEST_WIDTH = 6d;  // Smallest width of the robot in inches

    // Launch related constants

    // Hopefully linear value that will translate velocity of motors using motor.setVelocity()
    // to release velocity of the ball in meters per second.
    public static final double TICK_TO_RELEASE_VELOCITY = 0;
    public static final double GRAVITY = 9.807;  // m/s^2
    public static final double GOAL_HEIGHT = .9845;  // We want to aim for 7cm above this

    /* ---FORMULAS---
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