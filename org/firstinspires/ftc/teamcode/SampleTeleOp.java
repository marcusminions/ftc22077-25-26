

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="Demo Teleop", group="Linear OpMode")

public class SampleTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx arm = null;
    private DcMotorEx slides = null;
    private Servo roll = null;
    private Servo pitch = null;
    private Servo claw = null;
    private boolean isGrabbed = true;
    private double clawPos;
    private int stage = 0;
    
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft  = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm  = hardwareMap.get(DcMotorEx.class, "arm");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        pitch  = hardwareMap.get(Servo.class, "pitch");
        roll = hardwareMap.get(Servo.class, "roll");
        claw = hardwareMap.get(Servo.class, "claw");
        boolean rightroll = false;
        boolean leftroll = false;
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();
        runtime.reset();
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y*.7; // Remember, Y stick is reversed!
            double x = -gamepad1.left_stick_x*.7;
            double rx = -gamepad1.right_stick_x*.55;
            roll.setPosition(.27);

            frontLeft.setPower(y + x + rx);
            backLeft.setPower(y - x + rx);
            frontRight.setPower(y - x - rx);
            backRight.setPower(y + x - rx);
            //pitch.setPosition(0);
            //PITCH DISABLED SINCE WIRE CUT, MEHCANICALLY TIGHTED DO NOT UNCOMMENT
            
            claw.setPosition(clawPos);
            
            //stage = 0;  // See below
             //Ethan go Kys
            if(gamepad1.b)
                stage = 2;
            if(gamepad1.y)
                stage = 1;
            if(gamepad1.x)
                stage = 0;
            
            
            
            
            if(gamepad1.a)
            {
                isGrabbed = !isGrabbed;
                sleep(200);
            }
            
            if(isGrabbed) {
                clawPos = .33;
                telemetry.addLine("open");
            }
            else {
                clawPos = 0;
                telemetry.addLine("closed");
            }
            
             
            
            if(stage == 0)
            {
                slides.setTargetPosition(0);
                arm.setTargetPosition(-300);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                slides.setPower(1);
                //pitch.setPosition(0);
                //PITCH DISABLED SINCE WIRE CUT, MEHCANICALLY TIGHTED DO NOT UNCOMMENT

            }
            if(stage == 1)
            {
                arm.setTargetPosition(-800);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slides.setPower(-gamepad1.right_stick_y);
                slides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(.5);

                
            }
            if(stage == 2)
            {
                arm.setTargetPosition(-2000);
                slides.setPower(-gamepad1.right_stick_y);
                slides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                
                arm.setPower(1);
                
            }
            //balls
            
            // Show the elapsed game time and wheel powe``````````````````r.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("Pitch: " + pitch.getPosition());
            telemetry.addLine("Arm: " + arm.getCurrentPosition());
            telemetry.addLine("Stage" + stage);
            telemetry.update();
        }
    }
}
