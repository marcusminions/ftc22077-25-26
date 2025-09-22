// package MRILib.managers;

// import MRILib.managers.*;
// import MRILib.motion.*;
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import static MRILib.BotValues.*;

// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;

// public class PivotArmController implements Runnable {
//     private ArmBotFF bot;

//     private Telemetry telemetry;
    
//     private final double kG = 0.00089; //gravity gain //89
//     private final double kG2 = -0.003; //gravity gain 2 -.077
//     private final double g = .009810; // gravity constant in mm/second^2
    
//     private double currentSlidePosition = 0;
//     private double currentSlideLength = 350;
    
//     private double armPos = 0;
    
    
//     double m_slide  = 1.4; //kg
//     double m_claw   = .27; //kg
//     double m_motor  = .9; //kg
//     double d_motor  = 85; //mm
//     double d_slide = 350; //mm
//     double d_slideOffset = -85; //mm
    
    
    
//     double m_total = m_slide+m_claw+m_motor; //kg
//     double armCoM = (m_slide*(d_slide+d_slideOffset)/2 + m_motor*d_motor + m_claw*(d_slide+d_slideOffset))/m_total;
    
    
    
//     // private double slideM = 1.4;
//     // private double slideDfP = 160;
//     // private double motorM = .9;
//     // private double motorDfP = 80;
//     // private double clawM = .4;
//     // private double clawDfP = 270;
//     // private double totalM = slideM + motorM + clawM;
//     // private double armCoM = (slideM*slideDfP + motorM*motorDfP + clawM*clawDfP)/totalM;
//     private double armMountOffset = 90;
    
//     private double thetaOffset = 0; //set in botvalues
    
//     public enum PivotMode{
//         HOLD_POSITION,
//         RUN_TO_POSITION,
//         RUN_WITH_POWER
//     }
    
//     public volatile PivotMode pivotMode = PivotMode.RUN_TO_POSITION;
    
//     private volatile double inputPower = 0;
    
    
//     private PID positionPID = new PID(1.3, 0.01, 0.046); //2.5, 0.002, 0.23
//     //private PID velocityPID = new PID(1, 0, 1);

//     private volatile double targetTheta; //radians
//     private volatile double targetAngularVelocity;

//     private volatile double currentArmTheta;
//     private volatile double currentArmAngularVelocity;
//     private double previousArmTheta;
//     private double previousArmAngularVelocity;

//     private volatile boolean running = true;

//     public PivotArmController(ArmBotFF bot){
//         //constructor
//         this.bot = bot;
//         telemetry = bot.getTelemetry();
//     }


//     @Override 
//     public void run(){
//         long loopTime = 10;
//         long nextLoopTime = System.nanoTime() + loopTime * 1_000_000;
//         ElapsedTime timer = new ElapsedTime();
//         double previousTime = timer.seconds();
        
        
//         while(running){
            
//             //init variables
            
            
//             double currentTime = timer.seconds();
//             double deltaTime = currentTime-previousTime;
//             previousTime = currentTime;
//             double positionPIDValue = positionPID.update(currentArmTheta);
            
//             currentSlideLength = d_slide+(currentSlidePosition*DISTANCE_PER_TICK_SLIDE)+d_slideOffset;
//             armCoM = (m_slide*(currentSlideLength+d_slideOffset)/2 + m_motor*d_motor + m_claw*(d_slide+d_slideOffset))/m_total;


//             //thetaDot = (currentArmTheta - previousArmTheta) / deltaTime; //synching with discrete time
//             // double thetaDotDot = currentArmAngularVelocity / deltaTime;

//             // double thetaError = targetTheta - currentArmTheta;
//             // double angularVelocityError = targetAngularVelocity - thetaDotDot;
            
            
//             // FF compensation
            
//             double u_ff = calculateFeedForward(); //calculate u feedforward
            
            
//             // FB control
            
//             double error = Math.toDegrees(currentArmTheta)-Math.toDegrees(targetTheta);
//             double u_fb = positionPIDValue;
//             // //making sure it doesnt stall when close to target
//             if(Math.abs(error)>1){
//                 u_fb = Math.max(.1, Math.abs(u_fb));
//                 u_fb*=Math.signum(positionPIDValue);
//             }else{
//                 u_fb = 0;
//             }
            
//             if(u_fb<=0&&Math.toDegrees(currentArmTheta)>0){ //if going down
//                 u_fb*=.25;
//                 u_ff*=1.2;
//             }else if(u_fb<0){
//                 u_fb*=1.0;
//             }
            
//             //else u_fb*=1.12;
            
//             if(u_fb>0 && currentArmTheta>targetTheta){ //if going up
//                 u_fb=0;
//             }
//             else if(u_fb>0){
//                 u_fb*=.7;
//             }
            
//             if(currentArmTheta<0 && bot.getSlidePosSync()>100){
//                 u_ff *=2.2;
//             }
            
            
            
            
//             //power by mode
            
//             double u = 0;
//             switch(pivotMode){
//                 case RUN_TO_POSITION:
//                     double fbMulti = Math.min(.5, Math.PI/5/currentArmTheta*.5); //slowing down past ~65 degrees proportionately
//                     u = u_ff + u_fb;
//                     break;
//                 case HOLD_POSITION:
//                     u = u_ff;
//                     break;
//                 case RUN_WITH_POWER:
//                     u = u_ff*.3 + inputPower;
//                     break;
//             }
            
            
    
//             double power = setMotorPower(u);
            
            
//             if(false){
//                 telemetry.addData("PivotMode", pivotMode);
//                 telemetry.addData("inputPower", inputPower);
                
//                 telemetry.addData("arm encoder", armPos);
//                 telemetry.addData("theta", Math.toDegrees(currentArmTheta));
//                 telemetry.addData("target theta", Math.toDegrees(targetTheta));
//                 telemetry.addData("thetaOffset", thetaOffset);
//                 telemetry.addData("error", error);
                
//                 telemetry.addData("currentSlideLength", currentSlideLength);
                
                
//                 telemetry.addLine();
//                 // telemetry.addData("gravity ff1", gravityFeedForward1*(g) * (totalM));
//                 // telemetry.addData("gravity ff2", gravityFeedForward2*(g) * (totalM));
//                 // telemetry.addData("gravity ff", gravityFeedForward);
//                 //telemetry.addData("friction ff", frictionFeedforward);
//                 telemetry.addData("u ff", u_ff);
                
//                 telemetry.addLine();
//                 //telemetry.addData("pid pos", pidValue);
//                 telemetry.addData("pos pid", positionPIDValue);
//                 telemetry.addData("armCoM", armCoM);
//                 telemetry.addData("u fb", u_fb);
                
//                 telemetry.addLine();
//                 telemetry.addData("u", u);
//                 telemetry.addData("power output", power);
//                 telemetry.addData("deltaTime", deltaTime);
                
//                 telemetry.update();
//             }
            

//             while(System.nanoTime() < nextLoopTime){
//                 //wait until next loop time
//             }
//             nextLoopTime += loopTime * 1_000_000;
//         }
        
//     }

//     public double setMotorPower(double power){
//         //use voltage sensor to clamp volts to 0-12
//         double clampMulti = 12.0/bot.getVoltage();
//         //calculate 0-1 relative to current voltage output and the target voltage
//         double powerValue = (power)*clampMulti*.75;
//         //if(powerValue>0)powerValue*=1.5;
//         // if(powerValue<0 && Math.toDegrees(currentArmTheta)>0) powerValue*=.3; //slowing if going down so it dont slam
//         // else if(Math.toDegrees(currentArmTheta)>0) powerValue*=.8;
//         // else powerValue*=1.2;
        
//         // if(Math.toDegrees(currentArmTheta)>0){
//         //     if(powerValue<0) powerValue*=.36; //slowing if going down so it dont slam
//         //     else powerValue*=.8;
//         // }else{
//         //     if(targetTheta<=Math.toRadians(-12)){
//         //         powerValue = -.1;
//         //     }else{
//         //         if(powerValue<0) powerValue*=.36; //slowing if going down so it dont slam
//         //         else powerValue*=.8;
//         //     }
//         // }
        
        
//         bot.setPivotPower(Range.clip(powerValue, -.9, 1)); //apply power value to motor
//         return powerValue;
//     }
    
//     private double calculateFeedForward(){
//         double gravityFeedForward1 = kG * armCoM * Math.cos(currentArmTheta);
//         if(currentArmTheta-targetTheta>0 && gravityFeedForward1>0 &&Math.toDegrees(currentArmTheta)>0&&targetTheta>=0) gravityFeedForward1*=1.93;
//         else if(currentArmTheta>targetTheta && Math.toDegrees(currentArmTheta)<0)gravityFeedForward1*=.63;
//         double gravityFeedForward2 = kG2 * armMountOffset * Math.sin(currentArmTheta);
//         double gravityFeedForward = (gravityFeedForward1 + gravityFeedForward2);
//         return gravityFeedForward;
//     }

//     public synchronized double updateCurrentState(double armPos, double velocity, double slidePosition){
//         previousArmTheta = currentArmTheta;
//         previousArmAngularVelocity = currentArmAngularVelocity;
//         currentArmTheta = Math.toRadians(armPos/TICKS_PER_DEGREE_ARM)+thetaOffset;
//         this.armPos = armPos;
//         currentArmAngularVelocity = Math.toRadians(velocity/TICKS_PER_DEGREE_ARM); 
//         currentSlidePosition = slidePosition;
//         return Math.toDegrees(currentArmTheta);
//     }
//     public synchronized void updateCurrentState(double armPos, double velocity){
//         updateCurrentState(armPos, velocity, 0);
//     }
//     public synchronized void setPivotMode(PivotMode mode){
//         pivotMode = mode;
//     }
//     public synchronized void setInputPower(double input){
//         inputPower = input;
//     }

//     public void setTargetTheta(double theta, AngleUnit unit){
//         targetTheta = unit==AngleUnit.DEGREES?Math.toRadians(theta):theta;
//         positionPID.setTarget(targetTheta);
//     }
//     public void setTargetTheta(double theta)
//     { // overload to default to degrees if not specified
//         setTargetTheta(theta, AngleUnit.DEGREES);
//     }
//     public void setThetaOffset(double theta){
//         thetaOffset = Math.toRadians(theta);
//     }

//     public void stop(){
//         running = false;
//     }
// }
