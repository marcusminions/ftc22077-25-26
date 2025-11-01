package MRILib.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    public double kP; // software defined spring pulling toward target
    public double kI; // force away from repeated motion (anti wobble)
    public double kD; // software defined dampener pushing away from target
    public double iLimit = 1;

    public double targetVal = 0;
    public double minVal = 0;
    
    double errorSum = 0;
    public double errorSumTotal = .1;
    double lastTime = 0;
    double lastError = 0;
    
    public boolean isAngle;
    
    ElapsedTime timer = null;
    
    public PID(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
        timer = new ElapsedTime();
    }
    
    public void start(){
        errorSum = 0;
        lastError = 0;
        timer.reset();
    }
    
    public double update(double current){
        double error = targetVal - current;
        
        if (isAngle && error > 180) error -= 360;
        if (isAngle && error < -180) error += 360;
        
        double deltaTime = timer.seconds() - lastTime;

        if (Math.abs(error) < iLimit) {
            errorSum += error * deltaTime;
        }
        if(errorSum < -errorSumTotal)
            errorSum = -errorSumTotal;
        if(errorSum > errorSumTotal)
            errorSum = errorSumTotal;
        

        double errorRate = (error - lastError) / deltaTime;

        double value = kP * error + kI * errorSum + kD * errorRate;
        
        
        
        lastTime = timer.seconds();
        lastError = error;
        
        if (Math.abs(value) < minVal && value != 0) value = minVal * (value / Math.abs(value));
        return value;
    }
    
    public void setMinValue(double minimum) {
        minVal = minimum;
    }
    
    public double update(double current1, double current2){
        return (Math.abs(current1-targetVal) < Math.abs(current2-targetVal)?update(current1):update(current2));
    }
    
    public void setTarget(double target){
        targetVal = target;
    }
    
    public double getTarget() {
        return targetVal;
    }
    
    public void setILimit(double limit){
        iLimit = limit;
    }
    

}

