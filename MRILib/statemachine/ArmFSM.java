
/*
* This file contains the finite state machine for controlling your robot's arm with states and transitions
* 
* The init method contains the initialization of all states, which can then be accessed at runtime
* by DriveFSM in order to control the arm, or controlled by teleop with individual setState() calls
* 
* The protected class ArmState is implemented at the bottom, extending BotState with a transition system
* and a way to save initialized states for later use
* 
* In order for the state machine to react to dynamic information throughout the match (i.e. encoder changes and method calls),
* you must call armSM.update() in your opMode in the loop, after calling bot.update() to update the opMode member manager class
* 
* For a robot with a transfer system, or multiple arms, I suggest creating multiple ArmFSM classes and seperating the logic between them,
* as this allows for the easiest management of two seperate systems without making your state machines difficult to read and implement
* You can also make a manager class for both state machines that will call setState on both classes at once for QOL and readability
* 
* to see an example of the intended usage of this class, see MRILib.opmodes.AutonExample
*/


package MRILib.statemachine;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.HashMap;
import MRILib.managers.*;
import MRILib.util.*;
import java.util.function.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import MRILib.eventlistener.BotEventManager.*;
import MRILib.eventlistener.BotEventManager;
import MRILib.eventlistener.BotEvent;

import static MRILib.GameValues.*;
import MRILib.managers.LaunchController.LaunchMode;

public class ArmFSM {

    // Opmode member manager
    private LaunchBot bot; 

    // Currently active states
    public List<ArmState> currentStates = new ArrayList<>();

    // Mutually exclusive states
    public List<String[]> exclusiveStates = new ArrayList<String[]>();

    // Gamepads
    Bpad gpad1;
    Bpad gpad2;
    
    boolean auton;
    
    Telemetry telemetry;

    // Global variables
    boolean inLaunchZone = false;
    List<COLOR> inventory = new ArrayList<>();

    // Miscellaneous functions
    void detectIntake(BotEvent event) {
        if (inventory.size() > 2) inventory.remove(0);
        inventory.add(event.color);
    }

    void detectLaunch(BotEvent event) {
        if (inventory.size() > 0) inventory.remove(0);
    }
    
    // Constructor for Auton when gamepads are not needed
    public ArmFSM(LaunchBot bot, Telemetry telemetry) {
        auton = true;
        this.bot = bot;
        this.telemetry = telemetry;
        init();
        addState("DEFAULT");
    }

    // Constructor for TeleOp to take in gamepads
    public ArmFSM(LaunchBot bot, Bpad gpad1, Bpad gpad2, Telemetry telemetry) {
        auton = false;
        this.bot = bot;
        this.gpad1 = gpad1;
        this.gpad2 = gpad2;
        this.telemetry = telemetry;
        init();
        addState("DEFAULT"); // Starting on default state
    }

    public String toString(ArmState state) {
        return "" + state.toString();
    }

    // Runs once at the state's initialization
    public void start()               { start("DEFAULT"); }
    public void start(String name)    { start(ArmState.getState(name)); }
    public void start(ArmState state) { state.start(); }

    // Runs every repeat loop on the main thread
    public void update() {
        for (ArmState state : currentStates) {
            state.update();
            transition(state);
            checkExclusivity(state);
        }
    }

    // Runs once at the end of the state or before transitioning to another state
    public void end(String name) {
        end(ArmState.getState(name));
    }

    public void end(ArmState state) {
        if (currentStates.contains(state)) {
            state.end();
            currentStates.remove(state);
        }
    }

    // Adding another state to currentStates
    public void addState(String name) {
        addState(ArmState.getState(name));
    }

    public void addState(ArmState state) {
        if (!currentStates.contains(state)) {
            currentStates.add(state);
            state.start();
        }
    }

    // public void setState(String name) {
    //     setState(ArmState.getState(name));
    // }

    // // Setting currentState to another saved state
    // public void setState(ArmState state) { 
    //     if(state!=currentState && !state.toString().equals("TO_"+currentState.toString())) {
    //         currentState.end();
    //         currentState = state;
    //         state.start();
    //     }
    // }

    /*
     * Checks all transition conditions and transitions to the first valid target state.
     * Parallels are like conditions but they don't end the current state
     * This method should be called in the update() method of each state.
    */
    public void transition(ArmState state) {
        if (currentStates.contains(state)) {
            HashMap<String, Boolean> transitions = state.getTransitions();
            for(String targetState : transitions.keySet()) {
                if(transitions.get(targetState)) {
                    addState(targetState);
                    end(state);
                    currentStates.remove(state);
                    break;
                }
            }

            HashMap<String, Boolean> parallels = state.getParallels();
            for (String targetState : parallels.keySet()) {
                if (parallels.get(targetState)) {
                    addState(targetState);
                }
            }
        }
    }

    public void checkExclusivity(ArmState state) {
        boolean remove = false;

        // Check for exclusivity, if there is a conflict then remove the one that didn't come last
        for (String[] mutual : exclusiveStates) {
            if (Arrays.asList(mutual).contains(state.name)) {
                for (ArmState c : currentStates) {
                    if (Arrays.asList(mutual).contains(c.name) && currentStates.indexOf(c) > currentStates.indexOf(state)) {
                        remove = true;
                    }
                }
            }
        }

        // If marked for removal, end the state
        if (remove) state.end();
    }

    private void init() {
        // subscribe to broadcasts
        BotEventManager.subscribe(EventType.INTAKE, event -> detectIntake(event));
        BotEventManager.subscribe(EventType.LAUNCH, event -> detectLaunch(event));

        // Add mutually exclusive states
        exclusiveStates.add(new String[] {
            "DEFAULT", "AIM", "POWER", "FIRE"
        });

        // DEFAULT STATE
        new ArmState("DEFAULT") {
            @Override
            void update() {
                bot.setLaunchControllerMode(LaunchMode.OFF);
            }
        };

        new ArmState("AIM") {
            @Override
            void update() {
                bot.setLaunchControllerMode(LaunchMode.AIM);
            }
        };

        new ArmState("POWER") {
            @Override
            void update() {
                bot.setLaunchControllerMode(LaunchMode.POWER);
            }
        };

        new ArmState("FIRE") {
            @Override
            void update() {
                bot.setLaunchControllerMode(LaunchMode.FIRE);
            }
        };

        // Permanent states, usually logic for setting globals that isn't handled by broadcasts, start every name with P-
        new ArmState("P-LAUNCHZONE") {
            @Override
            void update() {
                Pose2D p = bot.getPosition();
                if (p.getX(DistanceUnit.INCH) + Math.abs(p.getY(DistanceUnit.INCH)) < WIDTH / 2) {  // Large zone
                    inLaunchZone = true;
                } else if (-p.getX(DistanceUnit.INCH) + 54 + Math.abs(p.getY(DistanceUnit.INCH)) < WIDTH / 2) {  // Small zone
                    inLaunchZone = true;
                } else {
                    inLaunchZone = false;
                }
            }
        };

        // Controls all logic and controls for teleop
        new ArmState("P-TELEOP") {
            @Override
            void update() {

            }
        };

        // Used to end a state without transitioning
        new ArmState("P-END") {
            @Override
            void update () {}
        }

    }
}

class ArmState extends BotState{

    // Static hashmap with all initialized states
    private static HashMap<String, ArmState> states = new HashMap<>();

    // Returning a previously saved state
    public static ArmState getState(String s) {
        try {
            return states.get(s);    
        } catch (Exception e) {
            return states.get("DEFAULT");
        }
    }

    // clearing the hashmap of all saved states for static garbage collecting
    public static void clearSavedStates() {
        states.clear();
    }

    // Constructors for ArmStates
    public ArmState() {
        super();
        states.put("DEFAULT", this);
    }
    public ArmState(String name) {
        super(name);
        states.put(name, this);
    }

    // Constructor that is optionally not saved in hashmap
    public ArmState(String name, boolean save) {
        super(name);
        if(save) states.put(name, this);
    }

    // Setting the default state to any initialized state
    public void setDefault(String state) {
        states.put("DEFAULT", states.get(state));
    }

    private HashMap<String, Boolean> transitions = new HashMap<>();
    private HashMap<String, Boolean> parallels = new HashMap<>();

    public void setTransition(String targetState, boolean condition) { transitions.put(targetState, condition); }
    public void setParallel(String targetState, boolean condition)   { parallels.put(targetState, condition); }

    public HashMap<String, Boolean> getTransitions() { return transitions; }
    public HashMap<String, Boolean> getParallels()   { return parallels; }
}

