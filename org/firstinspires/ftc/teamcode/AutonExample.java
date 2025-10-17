package org.firstinspires.ftc.teamcode;

import static MRILib.BotValues.*;
import MRILib.managers.*;
import MRILib.motion.*;
import MRILib.statemachine.*;
import java.util.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name = "AutonExample")
public class AutonExample extends LinearOpMode {

    public LaunchBot bot;
    public PIDController pid;
    public DriveFSM dsm;
    public ArmFSM asm;

    public enum SIDE { RED, BLUE, NONE }

    @Override
    public void runOpMode() {

        // General initialization
        bot = new LaunchBot(this);

        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);
        
        PID xPid = new PID(.7, .08, .02);
        PID yPid = new PID(.8, .08, .02);  // Something about friction for pDy > pDx
        PID thetaPid = new PID(1.5, .98, .09);
        thetaPid.errorSumTotal = .1;

        pid = new PIDController(bot, telemetry);
        pid.setPID(xPid, yPid);
        pid.setTurnPID(thetaPid);
        
        dsm = new DriveFSM(bot, pid, telemetry);
        asm = new ArmFSM(bot, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Setup for auto, defaults to red (50% chance of being correct?)
        SIDE side = SIDE.RED;
        int reflection = 0;  // ALL positions are just reflected across x axis, right?

        Pose2D redTarget = new Pose2D(DistanceUnit.INCH, -65, 65, AngleUnit.DEGREES, 0);
        Pose2D blueTarget = new Pose2D(DistanceUnit.INCH, -65, -65, AngleUnit.DEGREES, 0);

        // ADD CONTROLLER SETTINGS HERE
        while (opModeInInit()) {
            if (gamepad1.a) { side = SIDE.RED; }
            if (gamepad1.b) { side = SIDE.BLUE; }
            telemetry.addData("Side", side == SIDE.BLUE ? "BLUE" : "RED");
            telemetry.update();
        }

        waitForStart();
        
        // Side specific
        switch (side) {
        case NONE:
            // End program
            break;
        case RED:
            reflection = 1;
            bot.setLaunchControllerTarget(redTarget);
        case BLUE:
            reflection = 1;
            bot.setLaunchControllerTarget(blueTarget);
        }

        // State machine steps, positions
        Pose2D startPos = new Pose2D(DistanceUnit.INCH, 63, -12 * reflection, AngleUnit.DEGREES, -180 * reflection);
        Pose2D farLaunch = new Pose2D(DistanceUnit.INCH, 63, -12 * reflection, AngleUnit.DEGREES, -180 * reflection);
        Pose2D rightBallTop = new Pose2D(DistanceUnit.INCH, 36, -33 * reflection, AngleUnit.DEGREES, -90 * reflection);

        // Add permanent states
        asm.addState("P-LAUNCHZONE");

        bot.setPosition(startPos); 
        dsm.moveTo(startPos).run(() ->
        asm.addState("PREFIRE"));
        dsm.moveTo(farLaunch).run(() ->
        asm.addState("FIRE"));

        dsm.waitForSeconds(30);

        dsm.start();
        asm.start();
        bot.startMultiThread();
        
        // Main loop
        while (opModeIsActive()) {
            bot.update();
            dsm.update();
            asm.update();

            // For telemetry purposes
            Pose2D currentPos = bot.getPosition();

            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("Target Angle", pid.getTargetDegrees());
            telemetry.addData("Current Angle", pid.getCurrentDegrees());
            telemetry.addData("Target x", pid.getTargetX());
            telemetry.addData("Target y", pid.getTargetY());
            telemetry.addData("Current x", currentPos.getX(DistanceUnit.INCH));
            telemetry.addData("Current y", currentPos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", -bot.getHeading());
            telemetry.update();
        }

        bot.stopMultiThread();
    }
}