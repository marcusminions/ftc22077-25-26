package org.firstinspires.ftc.teamcode;

import static MRILib.BotValues.*;
import static MRILib.GameValues.*;

import MRILib.GameValues.COLOR;
import MRILib.managers.*;
import MRILib.motion.*;
import MRILib.statemachine.*;
import MRILib.util.Bpad;

import java.util.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name = "AAuton")
public class AutonExample extends LinearOpMode {

    public LaunchBot bot;
    public DriveFSM dsm;
    public ArmFSM asm;
    public PID xPid;
    public PID yPid;
    public PID wPid;

    @Override
    public void runOpMode() {

        // General initialization
        bot = new LaunchBot(this);

        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);
        
        PID xPid = new PID(.045, .0, .007);
        PID yPid = new PID(.049, .0, .007);  // Something about friction for pDy > pDx
        PID wPid = new PID(.014, .0, .0014);
        wPid.isAngle = true;

        wPid.errorSumTotal = .1;

        dsm = new DriveFSM(bot, xPid, yPid, wPid, telemetry);
        asm = new ArmFSM(bot, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Setup for auto, defaults to red (50% chance of being correct?)
        COLOR side = COLOR.RED;
        int reflection = 0;  // ALL positions are just reflected across x axis, right?

        Pose2D redTarget = new Pose2D(DistanceUnit.INCH, -65, 65, AngleUnit.DEGREES, 0);
        Pose2D blueTarget = new Pose2D(DistanceUnit.INCH, -65, -65, AngleUnit.DEGREES, 0);

        // ADD CONTROLLER SETTINGS HERE
        double startDelay = 0;
        while (opModeInInit()) {
            if (gamepad1.a || gamepad2.a) side = COLOR.RED;
            if (gamepad1.b || gamepad2.b) side = COLOR.BLUE;
            
            startDelay = 2;

            telemetry.addData("Side", side == COLOR.BLUE ? "BLUE" : "RED");
            telemetry.addData("Start Delay", startDelay);
            telemetry.update();
        }
        
        waitForStart();
        
        if (side == COLOR.RED) {
            reflection = 1;
            bot.setLaunchControllerTarget(redTarget);
        } else {
            reflection = -1;
            bot.setLaunchControllerTarget(blueTarget);
        }

        // State machine steps, positions
        Pose2D startPos = new Pose2D(DistanceUnit.INCH, 64, -16 * reflection, AngleUnit.DEGREES, 90);
        Pose2D midLaunch = new Pose2D(DistanceUnit.INCH, -5, -16 * reflection, AngleUnit.DEGREES, 90);
        Pose2D rightBallTop = new Pose2D(DistanceUnit.INCH, 36, -33 * reflection, AngleUnit.DEGREES, 180 * reflection);

        // Configure state machine variables
        asm.inventory.add(COLOR.PURPLE);
        asm.inventory.add(COLOR.PURPLE);
        asm.auton = true;
        
        // Add permanent states
        asm.addState("P-LAUNCHZONE");

        bot.setPosition(startPos); 
        dsm.waitForSeconds(startDelay);
        dsm.moveTo(midLaunch);
        dsm.waitForSeconds(1).run(() ->
        asm.addState("FIRE"));
        dsm.waitForSeconds(5);
        dsm.waitForSeconds(20);
        // dsm.moveTo(midLauch).run(() ->
        // asm.addState("FIRE"));
        // dsm.waitForSeconds(30);

        dsm.start();
        asm.start();
        // bot.startDriveThread();
        bot.startLaunchThread();
        
        // Main loop
        while (opModeIsActive()) {
            bot.update();
            dsm.update();
            asm.update();

            // For telemetry purposes
            Pose2D currentPos = bot.getPosition();

            telemetry.addLine();
            telemetry.addData("In Launch Zone?", asm.inLaunchZone);
            telemetry.addData("States", asm.currentStates);
            telemetry.addData("Drive State", dsm.steps.get(dsm.currentStep));
            telemetry.addLine();
            telemetry.addData("Target Angle", wPid.targetVal);
            telemetry.addData("Current Angle", bot.getHeading());
            telemetry.addData("Target x", xPid.targetVal);
            telemetry.addData("Target y", yPid.targetVal);
            telemetry.addData("Current x", currentPos.getX(DistanceUnit.INCH));
            telemetry.addData("Current y", currentPos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", bot.getHeading());
            telemetry.addLine();
            telemetry.update();
        }

        bot.stopDriveThread();
        bot.stopLaunchThread();
    }
}
