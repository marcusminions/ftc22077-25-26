package MRILib.opmodes;

import MRILib.util.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.*;

@TeleOp(name = "Motor Testing")
public class MotorTest extends OpMode{
    private Bpad gpad;

    DcMotorEx left;
    DcMotorEx right;

    double lVel;
    double rVel;
    double tlVel;
    double trVel;

    double increment;

    public void init(){
        gpad = new Bpad();

        lVel = 0; rVel = 0;
        tlVel = 0; trVel = 0;

        increment = 100;

        left = hardwareMap.get(DcMotorEx.class, "leftSpin");
        right = hardwareMap.get(DcMotorEx.class, "rightSpin");

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    int index = 0;
    public void loop(){
        gpad.update(gamepad1);
        
        lVel = left.getVelocity();
        rVel = right.getVelocity();

        if (gpad.get("dp_dpad_up") && gpad.get("left_bumper")) tlVel += increment;
        if (gpad.get("dp_dpad_up") && gpad.get("right_bumper")) trVel += increment;
        if (gpad.get("dp_dpad_left")) increment -= 10;
        if (gpad.get("dp_dpad_right")) increment += 10;

        if (gpad.get("left_trigger")) left.setVelocity(tlVel);
        else left.setVelocity(0);

        if (gpad.get("right_trigger")) right.setVelocity(trVel);
        else right.setVelocity(0);

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

        telemetry.update();
    }
}
