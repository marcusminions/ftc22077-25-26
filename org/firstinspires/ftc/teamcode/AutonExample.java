package org.firstinspires.ftc.teamcode;

import static MRILib.BotValues.*;
import com.qualcomm.robotcore.hardware.LED;
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
    public PIDController pid;
    public DriveFSM dsm;
    public ArmFSM asm;

    @Override
    public void runOpMode() {

        // General initialization
        bot = new LaunchBot(this);

        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);
        
        PID xPid = new PID(.041, .0, .0044);
        PID yPid = new PID(.044, .0, .0044);  // Something about friction for pDy > pDx
        PID thetaPid = new PID(.015, .0, .0012);
        
        // Configure PIDs
        // xPid.setMinValue(.01);
        // yPid.setMinValue(.01);
        thetaPid.isAngle = true;
        thetaPid.errorSumTotal = .1;

        pid = new PIDController(bot, telemetry);
        pid.setPID(xPid, yPid);
        pid.setTurnPID(thetaPid);
        
        dsm = new DriveFSM(bot, xPid, yPid, thetaPid, telemetry);
        asm = new ArmFSM(bot, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Setup for auto, defaults to red (50% chance of being correct?)
        COLOR side = COLOR.RED;
        int reflection = 0;  // ALL positions are just reflected across x axis, right?

        Pose2D redTarget = new Pose2D(DistanceUnit.INCH, -65, 65, AngleUnit.DEGREES, 0);
        Pose2D blueTarget = new Pose2D(DistanceUnit.INCH, -65, -65, AngleUnit.DEGREES, 0);

        // ADD CONTROLLER SETTINGS HERE
        boolean lastLB = false;
        boolean lastRB = false;
        double startDelay = 2;
        boolean farPos = true;
        boolean farShoot = false;
        while (opModeInInit()) {
            if (gamepad1.a || gamepad2.a) side = COLOR.RED;
            if (gamepad1.b || gamepad2.b) side = COLOR.BLUE;
            if (gamepad1.dpad_left || gamepad2.dpad_left) { farPos = false; farShoot = false; }
            if (gamepad1.dpad_right || gamepad2.dpad_right) farPos = true;
            if (gamepad1.dpad_up || gamepad2.dpad_up) farShoot = true;
            
            if ((gamepad1.left_bumper || gamepad2.left_bumper) && !lastLB) startDelay -= 1;
            if ((gamepad1.right_bumper || gamepad2.right_bumper) && !lastRB) startDelay += 1;
            
            lastLB = (gamepad1.left_bumper || gamepad2.left_bumper) ? true : false;
            lastRB = (gamepad1.right_bumper || gamepad2.right_bumper) ? true : false;
            
            telemetry.addLine("---CONTROLS---");
            telemetry.addLine("Side: A=red, B=blue");
            telemetry.addLine("Start Delay: lb=decrease, rb=increase");
            telemetry.addLine("Start Position: dpad left=goal, dpad right=far");
            telemetry.addLine("Shoot Position: dpad up=far");
            telemetry.addLine(" ");
            telemetry.addData("Side", side == COLOR.BLUE ? "BLUE" : "RED");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("Position = far?", farPos);
            telemetry.addData("Shoot = far?", farShoot);
            telemetry.update();
        }
        
        waitForStart();
        
        bot.resetIMUHeading();
        
        if (side == COLOR.RED) {
            reflection = -1;
            bot.setLaunchControllerTarget(redTarget);
        } else {
            reflection = 1;
            bot.setLaunchControllerTarget(blueTarget);
        }

        // State machine steps, positions
        Pose2D goalStart = new Pose2D(DistanceUnit.INCH, -62, -42 * reflection, AngleUnit.DEGREES, -90);
        Pose2D farStart = new Pose2D(DistanceUnit.INCH, 63, -11 * reflection, AngleUnit.DEGREES, -90);
        Pose2D farLaunch = new Pose2D(DistanceUnit.INCH, 59, -11 * reflection, AngleUnit.DEGREES, -90 + 20*reflection);
        Pose2D midLaunch = new Pose2D(DistanceUnit.INCH, -11, -12 * reflection, AngleUnit.DEGREES, -90 + 45*reflection);
        Pose2D farBall = new Pose2D(DistanceUnit.INCH, 36, -40 * reflection, AngleUnit.DEGREES, -90 - 90*reflection);
        Pose2D endPos = new Pose2D(DistanceUnit.INCH, 36, -24 * reflection, AngleUnit.DEGREES, -90 - 90*reflection);

        // Configure state machine variables
        asm.inventory.add(COLOR.PURPLE);
        asm.inventory.add(COLOR.PURPLE);
        asm.inventory.add(COLOR.PURPLE);
        asm.auton = true;
        
        // Add permanent states
        asm.addState("P-LAUNCHZONE");
        
        if (!farShoot) {
            if (!farPos) bot.setPosition(goalStart);
            else bot.setPosition(farStart);
    
            dsm.waitForSeconds(startDelay);
            dsm.moveTo(midLaunch);
            dsm.waitForSeconds(1).run(() ->
            asm.addState("FIRE"));
            dsm.waitForSeconds(10);
            dsm.moveTo(endPos);
            dsm.waitForSeconds(20);
            // dsm.moveTo(midLauch).run(() ->
            // asm.addState("FIRE"));
            // dsm.waitForSeconds(30);
        } else {
            bot.setPosition(farStart);
            bot.setLaunchControllerFirePosition(LaunchController.LaunchMode.FAR);
            
            dsm.waitForSeconds(startDelay);
            dsm.moveTo(farLaunch);
            dsm.waitForSeconds(1).run(() ->
            asm.addState("FIRE"));
            dsm.waitForSeconds(8);
            dsm.moveTo(endPos);
            dsm.moveTo(farBall).run(() ->
            asm.addState("INTAKE"));
            dsm.moveTo(farLaunch);
            dsm.waitForSeconds(1).run(() ->
            asm.end("INTAKE")).run(() ->
            asm.addState("FIRE"));
            dsm.waitForSeconds(8);
            dsm.moveTo(endPos);
        }
        
        dsm.waitForSeconds(20);

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
            telemetry.addData("States", asm.currentStates);
            telemetry.addData("Drive State", dsm.steps.get(dsm.currentStep));
            telemetry.addLine();
            telemetry.addData("Target Angle", thetaPid.getTarget());
            telemetry.addData("Current Angle", thetaPid.getTarget());
            telemetry.addData("Target x", xPid.getTarget());
            telemetry.addData("Target y", yPid.getTarget());
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
