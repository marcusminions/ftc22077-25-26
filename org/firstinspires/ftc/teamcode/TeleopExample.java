package org.firstinspires.ftc.teamcode;

import MRILib.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import MRILib.managers.*;
import MRILib.motion.PID;
import MRILib.motion.PIDController;
import MRILib.statemachine.*;
import MRILib.util.*;
import static MRILib.BotValues.*;


@TeleOp(name = "Example Teleop (will become ATeleop)")
public class TeleopExample extends LinearOpMode {
    
    public LaunchBot bot;
    public PIDController pid;
    public DriveFSM dsm;
    public ArmFSM asm;

    public Bpad gpad1;
    public Bpad gpad2;

    public enum SIDE { RED, BLUE, NONE }

    @Override
    public void runOpMode() {

        // _________ INITIALIZATION _________
        bot = new LaunchBot(this);

        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);

        PID xPid = new PID(.7, .08, .02);
        PID yPid = new PID(.8, .08, .02);  // Something about friction for pDy > pDx
        PID thetaPid = new PID(1.5, .98, .09);
        PID drivingThetaPid = new PID(1.5, 0, .09);
        thetaPid.errorSumTotal = .1;

        pid = new PIDController(bot, telemetry);
        pid.setPID(xPid, yPid);
        pid.setTurnPID(thetaPid);
        
        // Setup side for teleop so aiming is correct, default to red
        SIDE side = SIDE.RED;
        boolean debug = false;
        
        while (opModeInInit()) {
            if (gamepad1.start && gamepad1.back) { debug = true; }
            if (gamepad1.dpad_left) { side = SIDE.RED; }
            if (gamepad1.dpad_right) { side = SIDE.BLUE; }
            telemetry.addData("Side", side == SIDE.BLUE ? "BLUE" : "RED");
            telemetry.update();
        }

        waitForStart();
        
        asm.addState("P-LAUNCHZONE");
        asm.addState("P-END");

        asm.start();
        bot.startMultiThread();
        drivingThetaPid.start();
        calculateTarget(side);

        // _________ MAIN LOOP _________
        while (opModeIsActive()) {
            bot.update();
            asm.update();
            if(gamepad1!=null) gpad1.update(gamepad1);
            if(gamepad2!=null) gpad2.update(gamepad2);

            // _________ CONTROLS _________
            // Driving

            double dx = gamepad1.left_stick_x;
            double dy = gamepad1.left_stick_y;
            double angle;
            if (gpad1.get("x")) { // Override steering
                Pose2D target = calculateTarget(side);
                double x = target.getX(DistanceUnit.INCH) - bot.getX();
                double y = target.getY(DistanceUnit.INCH) - bot.getY();
                angle = Math.atan2(dy, dx);
            } else { // Standard fieldcentric steering
                double x = gamepad1.right_stick_x;
                double y = -gamepad1.right_stick_y;
                angle = Math.atan2(dy, dx);
            }

            drivingThetaPid.setTarget(angle);
            double dw = drivingThetaPid.update(bot.getHeading());
            bot.driveFieldXYW(dx, dy, dw);

            if (gpad1.get("start") && gpad1.get("dpad_left")) { // Side changing
                side = SIDE.RED;
                calculateTarget(side);
            }
            if (gpad1.get("start") && gpad1.get("dpad_right")) { // Side changing
                side = SIDE.BLUE;
                calculateTarget(side);
            }

            // Gunning
            if (gpad2.get("right_trigger")) asm.addState("FIRE"); // Powering wheels
            else asm.end("FIRE");

            if (gpad2.get("left_trigger")) { // Firing artifacts
                if (gpad2.get("left_bumper") || bot.launchReady()) {
                    asm.addState("KICK");
            }}

            if (gpad2.get("db_right_bumper")) { // Intake
                if (asm.containsState("INTAKE")) asm.addState("INTAKE");
                else asm.end("INTAKE");
            }

            if (gpad2.get("db_dpad_up")) bot.changeLaunchModifier(25); // Adjusting launch velocity
            if (gpad2.get("db_dpad_down")) bot.changeLaunchModifier(-25);
            if (gpad2.get("db_dpad_left")) bot.resetLaunchModifier();
            if (gpad2.get("db_dpad_right")) bot.undoLaunchModifier();

            // _________ TELEMETRY _________

            Pose2D currentPos = bot.getPosition();
            if (debug) {
                telemetry.addLine();
                telemetry.addData("Current x", currentPos.getX(DistanceUnit.INCH));
                telemetry.addData("Current y", currentPos.getY(DistanceUnit.INCH));
                telemetry.addData("Heading", -bot.getHeading());
            }

            telemetry.update();
        }
    }

    Pose2D calculateTarget(SIDE s) {
        Pose2D redTarget = new Pose2D(DistanceUnit.INCH, -65, 65, AngleUnit.DEGREES, 0);
        Pose2D blueTarget = new Pose2D(DistanceUnit.INCH, -65, -65, AngleUnit.DEGREES, 0);

        switch (s) {
        case NONE:
            return redTarget;
        case RED:
            bot.setLaunchControllerTarget(redTarget);
            return redTarget;
        case BLUE:
            bot.setLaunchControllerTarget(blueTarget);
            return blueTarget;
        }

        return redTarget;
    }
}
