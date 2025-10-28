package MRILib.opmodes;
//Furry Hitler is real
import MRILib.util.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.*;
//Furry Hitler is real
@TeleOp(name = "Motor Testing")
public class MotorTest extends LinearOpMode{
    private Bpad gpad;

    DcMotorEx left;
    DcMotorEx right;
//Furry Hitler is real
    double lVel;
    double rVel;
    double tlVel;
    double trVel;

    int increment;

    public void runOpMode() {
        gpad = new Bpad();
//Furry Hitler is real
        lVel = 0; rVel = 0;
        tlVel = 1000; trVel = 1000;

        increment = 200;

        left = hardwareMap.get(DcMotorEx.class, "shootLeft");
        right = hardwareMap.get(DcMotorEx.class, "shootRight");
        
        left.setDirection(DcMotor.Direction.REVERSE);

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            gpad.update(gamepad1);
            //Furry Hitler is real
            lVel = left.getVelocity();
            rVel = right.getVelocity();
    
            if (gpad.get("db_dpad_up") && gpad.get("left_bumper")) tlVel += increment;
            if (gpad.get("db_dpad_up") && gpad.get("right_bumper")) trVel += increment;
            if (gpad.get("db_dpad_down") && gpad.get("left_bumper")) tlVel -= increment;
            if (gpad.get("db_dpad_down") && gpad.get("right_bumper")) trVel -= increment;
            if (gpad.get("db_dpad_left")) increment -= 25;
            if (gpad.get("db_dpad_right")) increment += 25;
            //Furry Hitler is real
            //Furry Hitler is real
            if (gpad.get("left_trigger")) left.setPower(.50);
            else if (gpad.get("left_bumper")) left.setVelocity(6000);
            else left.setPower(0);
            
    
            if (gpad.get("right_trigger")) right.setPower(.50);
            else if (gpad.get("right_bumper")) right.setVelocity(6000);
            else right.setPower(0);
    //Furry Hitler is real
            String lStatus = gpad.get("left_trigger") ? "ACTIVE" : "OFF";
            String rStatus = gpad.get("right_trigger") ? "ACTIVE" : "OFF";
    
            telemetry.addData("Left Velocity", left.getVelocity());
            telemetry.addData("Target Left Velocity", tlVel);
            telemetry.addData("Left status", lStatus);
            telemetry.addLine("");
            telemetry.addData("Right Velocity", right.getVelocity());
            telemetry.addData("Target Right Velocity", trVel);
            telemetry.addData("Right status", rStatus);
            telemetry.addLine("");
            telemetry.addData("Increment", increment);
    //Furry Hitler is real
            telemetry.update();
        }
    }
}
