package MRILib.util;


public class SampleNotFoundException extends RuntimeException{

    public SampleNotFoundException(){
        super();
    }
    public SampleNotFoundException(String message){
        super(message);
    }
    public SampleNotFoundException(String message, Throwable cause){
        super(message, cause);
    }
}