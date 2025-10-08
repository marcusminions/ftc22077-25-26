package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TestFullBot")
public class TestFullBot extends LinearOpMode {

    // Drivetrain Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Xtra Motors
    private DcMotor sweeperMotor;
    private DcMotor conveyorMotor;
    private DcMotorEx shootLeft;
    private DcMotorEx shootRight;


    //Toggles
    private boolean sweeperOn = false;
    private boolean conveyorOn = false;
    private boolean geckoWheelsOn = false;

    // button press tracking thingy
    private boolean aButtonPreviouslyPressed = false;
    private boolean bButtonPreviouslyPressed = false;
    private boolean xButtonPreviouslyPressed = false;


    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        sweeperMotor = hardwareMap.get(DcMotor.class, "sweeperMotor");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        shootLeft = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; 
            double x = gamepad1.left_stick_x * 1.1; 
            double rx = gamepad1.right_stick_x; 

            
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;
            
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower(frontLeftPower / denominator);
            backLeft.setPower(backLeftPower / denominator);
            frontRight.setPower(frontRightPower / denominator);
            backRight.setPower(backRightPower / denominator);


            

            // Toggle Sweeper Motor (A button)
            if (gamepad1.a && !aButtonPreviouslyPressed) {
                sweeperOn = !sweeperOn; 
            }
            aButtonPreviouslyPressed = gamepad1.a; 

            // Toggle Conveyor Motor (B button)
            if (gamepad1.b && !bButtonPreviouslyPressed) {
                conveyorOn = !conveyorOn;
            }
            bButtonPreviouslyPressed = gamepad1.b; 

            // Toggle Gecko Wheels Motor (X button)
            if (gamepad1.x && !xButtonPreviouslyPressed) {
                geckoWheelsOn = !geckoWheelsOn; 
            }
            xButtonPreviouslyPressed = gamepad1.x;



            //sweeper power  on its toggle state
            if (sweeperOn) {
                sweeperMotor.setPower(.6);
            } else {
                sweeperMotor.setPower(0);
            }

            //conveyor power based on its toggle state
            if (conveyorOn) {
                conveyorMotor.setPower(.6);
            } else {
                conveyorMotor.setPower(0);
            }
            
            //gecko wheels power based on its toggle state
            if (geckoWheelsOn && gamepad1.left_bumper) {
                shootLeft.setPower(gamepad1.left_trigger);
                shootRight.setPower(0.8 * gamepad1.left_trigger);
            } else if(geckoWheelsOn) {
                shootLeft.setPower(gamepad1.left_trigger);
                shootRight.setPower(gamepad1.left_trigger);
            }
            else {
                shootLeft.setPower(0);
                shootRight.setPower(0);
            }


           
            telemetry.addData("Sweeper", sweeperOn ? "ON" : "OFF");
            telemetry.addData("Conveyor", conveyorOn ? "ON" : "OFF");
            telemetry.addData("Gecko Wheels", shootRight.getPower());
            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.addData("Front Right Power", frontRight.getPower());
            telemetry.update();
        }
    }
}