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
import static MRILib.GameValues.*;


@TeleOp(name = "Example Teleop (will become ATeleop)")
public class TeleopExample extends LinearOpMode {
    
    public LaunchBot bot;
    public PIDController pid;
    public DriveFSM dsm;
    public ArmFSM asm;

    private Bpad gpad1;
    private Bpad gpad2;

    @Override
    public void runOpMode() {

        // _________ INITIALIZATION _________
        gpad1 = new Bpad();
        gpad2 = new Bpad();
        
        bot = new LaunchBot(this);
        asm = new ArmFSM(bot, gpad1, gpad2, telemetry);

        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);

        PID xPid = new PID(.7, .08, .02);
        PID yPid = new PID(.8, .08, .02);  // Something about friction for pDy > pDx
        PID thetaPid = new PID(1.5, .98, .09);
        PID drivingThetaPid = new PID(.014, 0, .0014);
        thetaPid.errorSumTotal = .1;
        drivingThetaPid.isAngle = true;

        pid = new PIDController(bot, telemetry);
        pid.setPID(xPid, yPid);
        pid.setTurnPID(thetaPid);
        
        dsm = new DriveFSM(bot, pid, telemetry);
        
        // Setup side for teleop so aiming is correct, default to red
        COLOR side = COLOR.RED;
        
        double angle = 0;
        boolean debug = false;
        
        while (opModeInInit()) {
            if (gamepad1.start && gamepad1.back) { debug = true; }
            if (gamepad1.dpad_left) { side = COLOR.RED; }
            if (gamepad1.dpad_right) { side = COLOR.BLUE; }
            telemetry.addData("Side", side == COLOR.BLUE ? "BLUE" : "RED");
            telemetry.update();
        }

        waitForStart();
        
        asm.addState("P-LAUNCHZONE");
        asm.addState("P-END");

        asm.start();
        bot.startMultiThread();
        drivingThetaPid.start();
        calculateTarget(side);
        
        // _________ CREATE OTHER VARIABLES _________
        boolean lastGpadX = false;
        boolean lastGpadA = false;
        boolean lastGpadB = false;
        
        boolean flyWheelToggle = false;
        boolean intakeToggle = false;
        boolean beltToggle = false;
        
        // _________ MAIN LOOP _________
        while (opModeIsActive()) {
            bot.update();
            asm.update();
            if(gamepad1!=null) gpad1.update(gamepad1);
            if(gamepad2!=null) gpad2.update(gamepad2);

            // _________ CONTROLS _________
            // Utility
            if (gpad1.get("start") && gpad1.get("dpad_left")) { side = COLOR.RED; calculateTarget(side); }
            if (gpad2.get("start") && gpad2.get("dpad_left")) { side = COLOR.RED; calculateTarget(side); }
            if (gpad1.get("start") && gpad1.get("dpad_right")) { side = COLOR.BLUE; calculateTarget(side); }
            if (gpad2.get("start") && gpad2.get("dpad_right")) { side = COLOR.BLUE; calculateTarget(side); }

            // Driving
            double dx = gamepad1.left_stick_y;
            double dy = gamepad1.left_stick_x;
            if (gpad1.get("x")) { // Override steering
                Pose2D target = calculateTarget(side);
                double x = target.getX(DistanceUnit.INCH) - bot.getX();
                double y = target.getY(DistanceUnit.INCH) - bot.getY();
                angle = Math.atan2(y, x) * 180 / Math.PI;
            } else { // Standard fieldcentric steering
                double x = -gamepad1.right_stick_x;
                double y = gamepad1.right_stick_y;
                if (Math.sqrt(x*x + y*y) > .8) angle = Math.atan2(x, y) * 180 / Math.PI;
            }

            drivingThetaPid.setTarget(angle);
            double dw = drivingThetaPid.update(bot.getHeading());
            bot.driveFieldXYW(dx, dy, dw);
            
            if (gpad1.get("start") && gpad1.get("dpad_up")) bot.resetHeading();

            // Gunning
            // b belt, a intake, make flywheels brake
            // lt turns on flywheels power to trigger, x is flywheel safety toggle
            // kicker is default open (back) lb kicks forward when held
            // dpad left is reverse intake, dpad up/down intake power .2, right dpad resets
            if (gamepad2.x && !lastGpadX) flyWheelToggle = !flyWheelToggle;
            if (gamepad2.a && !lastGpadA) intakeToggle = !intakeToggle;
            if (gamepad2.b && !lastGpadB) beltToggle = !beltToggle;
            
            if (intakeToggle) bot.setIntakePower(1);
            
            if (flyWheelToggle) {
                bot.setLeftPower(gamepad2.left_trigger);
                bot.setRightPower(gamepad2.left_trigger);
            }
            
            // if (gamepad1)
            
            if (bot.launchReady()) gamepad2.rumble(75); // Feedback if able to kick

            if (gpad2.get("left_trigger")) { // Firing artifacts
                if (gpad2.get("left_bumper") || bot.launchReady()) {
                    asm.addState("KICK");
            }}

            if (gpad2.get("db_right_bumper")) { // Intake
                if (asm.containsState("INTAKE")) asm.end("INTAKE");
                else asm.addState("INTAKE");
            }

            if (gpad2.get("db_dpad_up")) bot.changeLaunchModifier(25); // Adjusting launch velocity
            if (gpad2.get("db_dpad_down")) bot.changeLaunchModifier(-25);
            if (gpad2.get("db_dpad_left")) bot.resetLaunchModifier();
            if (gpad2.get("db_dpad_right")) bot.undoLaunchModifier();
            
            // More utility
            if (gamepad2.x) lastGpadX = true; else lastGpadX = false;
            if (gamepad2.a) lastGpadA = true; else lastGpadA = false;
            if (gamepad2.b) lastGpadB = true; else lastGpadB = false;

            // _________ TELEMETRY _________
            telemetry.addData("Heading", -bot.getHeading());
            telemetry.addData("Angle", angle);
            telemetry.addData("Side", side);
            telemetry.addLine("-----------------------");
            telemetry.addData("Left velocity", bot.getLeftVelocity());
            telemetry.addData("Right velocity", bot.getRightVelocity());
            telemetry.addLine("-----------------------");
            telemetry.addData("States", asm.currentStates);

            Pose2D currentPos = bot.getPosition();
            if (debug) {
                telemetry.addLine("--------DEBUG--------");
                telemetry.addData("Current x", currentPos.getX(DistanceUnit.INCH));
                telemetry.addData("Current y", currentPos.getY(DistanceUnit.INCH));
            }

            telemetry.update();
        }
    }

    Pose2D calculateTarget(COLOR s) {
        Pose2D redTarget = new Pose2D(DistanceUnit.INCH, -65, 65, AngleUnit.DEGREES, 0);
        Pose2D blueTarget = new Pose2D(DistanceUnit.INCH, -65, -65, AngleUnit.DEGREES, 0);

        if (s == COLOR.BLUE) {
            bot.setLaunchControllerTarget(blueTarget);
            return blueTarget;
        } else {
            bot.setLaunchControllerTarget(redTarget);
            return redTarget;
        }
    }
}
