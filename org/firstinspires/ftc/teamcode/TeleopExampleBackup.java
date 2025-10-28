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


@TeleOp(name = "Example Teleop (Backup)")
public class TeleopExampleBackup extends LinearOpMode {
    
   
    public LaunchBot bot;
    public PIDController pid;
    public DriveFSM dsm;
    // public ArmFSM asm;

    @Override
    public void runOpMode() {

        // _________ INITIALIZATION _________
        bot = new LaunchBot(this);
        // asm = new ArmFSM(bot, gpad1, gpad2, telemetry);

        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);

        PID xPid = new PID(.7, .08, .02);
        PID yPid = new PID(.8, .08, .02);  // Something about friction for pDy > pDx
        PID thetaPid = new PID(.014, 0, .0014);
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
        
        // asm.addState("P-LAUNCHZONE");
        // asm.addState("P-END");

        // asm.start();
        // bot.startMultiThread();
        Pose2D startPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        bot.setPosition(startPos);
        drivingThetaPid.start();
        calculateTarget(side);
        
        // _________ CREATE OTHER VARIABLES _________
        boolean lastGpadX = false;
        boolean lastGpadA = false;
        boolean lastGpadB = false;
        boolean lastDpadL = false;
        
        double intakePower = .8;
        
        boolean flyWheelToggle = false;
        boolean intakeToggle = false;
        boolean beltToggle = false;
        int reverseToggle = 1;
        
        // _________ MAIN LOOP _________
        while (opModeIsActive()) {
            bot.update();
            // asm.update();

            // _________ CONTROLS _________
            // Utility
            if (gamepad1.start && gamepad1.dpad_left) { side = COLOR.RED; calculateTarget(side); }
            if (gamepad1.start && gamepad1.dpad_left) { side = COLOR.RED; calculateTarget(side); }
            if (gamepad1.start && gamepad1.dpad_right) { side = COLOR.BLUE; calculateTarget(side); }
            if (gamepad1.start && gamepad1.dpad_right) { side = COLOR.BLUE; calculateTarget(side); }

            // Driving
            double dx = gamepad1.left_stick_y;
            double dy = gamepad1.left_stick_x;
            if (gamepad1.x) { // Override steering
                Pose2D target = calculateTarget(side);
                double x = target.getX(DistanceUnit.INCH) - bot.getX();
                double y = target.getY(DistanceUnit.INCH) - bot.getY();
                angle = Math.atan2(y, x) * 180 / Math.PI;
            } else { // Standard fieldcentric steering
                double x = gamepad1.right_stick_x;
                double y = gamepad1.right_stick_y;
                if (Math.sqrt(x*x + y*y) > .8) angle = Math.atan2(x, y) * 180 / Math.PI;
            }

            drivingThetaPid.setTarget(angle);
            double dw = drivingThetaPid.update(bot.getHeading());
            bot.driveFieldXYW(dx, dy, dw);
            
            if (gamepad1.start && gamepad1.dpad_up) bot.resetHeading();

            // Gunning
            // b belt, a intake, make flywheels brake
            // lt turns on flywheels power to trigger, x is flywheel safety toggle
            // kicker is default open (back) lb kicks forward when held
            // dpad left is reverse intake, dpad up/down intake power .2, right dpad resets
            if (gamepad2.x && !lastGpadX) flyWheelToggle = !flyWheelToggle;
            if (gamepad2.a && !lastGpadA) intakeToggle = !intakeToggle;
            if (gamepad2.b && !lastGpadB) beltToggle = !beltToggle;
            if (gamepad2.dpad_left && !lastDpadL) reverseToggle *= -1;
            
            if (gamepad2.dpad_up && intakePower <= .8) intakePower += .2;
            if (gamepad2.dpad_down && intakePower >= .2) intakePower -= .2;
            if (gamepad2.dpad_right) intakePower = .8;
            
            if (gamepad2.left_bumper) bot.setKickerPosition(0);
            else bot.setKickerPosition(.10);
            
            if (intakeToggle) bot.setIntakePower(intakePower * reverseToggle);
            else bot.setIntakePower(0);
            
            if (beltToggle) bot.setConveyorPower(1);
            else bot.setConveyorPower(0);
            
            if (flyWheelToggle) {
                bot.setLeftVelocity(gamepad2.left_trigger * 1850);
             
                bot.setRightVelocity(gamepad2.left_trigger * 1850);
                gamepad2.rumble(75);
            } else {
                bot.setLeftPower(0);
                bot.setRightPower(0);
            }
            
            // More utility
            if (gamepad2.x) lastGpadX = true; else lastGpadX = false;
            if (gamepad2.a) lastGpadA = true; else lastGpadA = false;
            if (gamepad2.b) lastGpadB = true; else lastGpadB = false;
            if (gamepad2.dpad_left) lastDpadL = true; else lastDpadL = false;

            // _________ TELEMETRY _________
            telemetry.addData("Heading", -bot.getHeading());
            telemetry.addData("Angle", angle);
            telemetry.addData("Side", side);
            telemetry.addLine("-----------------------");
            telemetry.addData("Left velocity", bot.getLeftVelocity());
            telemetry.addData("Right velocity", bot.getRightVelocity());
            telemetry.addLine("-----------------------");
            telemetry.addData("last a", flyWheelToggle);
            telemetry.addData("left trigger", gamepad2.left_trigger);
            // telemetry.addData("States", asm.currentStates);

            Pose2D currentPos = bot.getPosition();
            telemetry.addLine("--------DEBUG--------");
            telemetry.addData("Current x", currentPos.getX(DistanceUnit.INCH));
            telemetry.addData("Current y", currentPos.getY(DistanceUnit.INCH));
            
            if(gamepad2.guide) {
                this.calculateTarget(COLOR.BLUE);
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
