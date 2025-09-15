package MRILib.opmodes;

import MRILib.util.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.*;

@TeleOp(name = "Servo Zero")
public class ServoZero extends OpMode{
    private Bpad gpad;

    Servo pitch;
    Servo roll;
    Servo claw;

    Servo[] servoList = new Servo[3];
    double[] positionList;

    

    public void init(){
        gpad = new Bpad();
        claw = hardwareMap.get(Servo.class, "claw");
        pitch = hardwareMap.get(Servo.class, "pitch");
        roll = hardwareMap.get(Servo.class, "roll");
        servoList[0] = pitch;
        servoList[1] = roll;
        servoList[2] = claw;
        positionList = new double[3];

    }

    int index = 0;
    public void loop(){
        
        claw.setPosition(.3);
        
        gpad.update(gamepad1);
        
        if(gpad.get("db_dpad_down")&&index<servoList.length-1) index++;
        if(gpad.get("db_dpad_up")&&index>0) index--;

        if(gpad.get("db_y"))positionList[index]+=.1;
        if(gpad.get("db_a"))positionList[index]-=.1;

        if(positionList[index]>1)positionList[index]=0;
        if(positionList[index]<0)positionList[index]=1;

        if(gpad.get("left_trigger")){
            for(int i=0;i<servoList.length;i++){
                servoList[i].setPosition(positionList[i]);
            }
        }
        if(gpad.get("right_trigger"))servoList[index].setPosition(positionList[index]);

        telemetry.addData((index==0?">":" ")+"pitch", positionList[0]);
        telemetry.addData((index==1?">":" ")+"roll", positionList[1]);
        telemetry.addData((index==2?">":" ")+"claw", positionList[2]);
        telemetry.update();
    }
}
