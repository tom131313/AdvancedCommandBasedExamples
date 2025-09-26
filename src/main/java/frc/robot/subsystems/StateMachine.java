package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// @SuppressWarnings("unused") //FIXME remove
/**
 * Example of a Finite State Machine (FSM) using simple methods to build the FSM from those utility
 * class.
 * 
 * Command-Based classes are used to wrap the users commands and triggers in order to define the
 * FSM "cyclic" behavior.
 */
public class StateMachine extends Command {
  ///////////////////////////////////////////
  // THE CONTROL COMMAND OF THE STATE MACHINE
  ///////////////////////////////////////////
  
  boolean FSMfinished;
  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {
    // System.out.println("StateMachine initialize");
    FSMfinished = false;
    new ScheduleCommand(initialState.stateCommandAugmented).schedule();
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  public void execute() {
    // System.out.println("StateMachine execute");
    events.poll();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.

   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {
    // System.out.println("StateMachine end");
   }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  public boolean isFinished() {
    return FSMfinished;
  }

/////////////////////////////////////
// RUN THE WRAPPED STATES AS COMMANDS
/////////////////////////////////////

private class WrapState extends WrapperCommand {
  WrapState(Command command) {
    super(command); // should be state command to run at this time
  }

  public void initialize() {
    // System.out.println("Wrapper initialize");
    events.clear(); // wipe the previous state's triggers
    // stateCompleted = false;

      State currentState = null;
      if (currentTransitionID != 0) {
      // find the immediately preceding transition that got us here using currentTransitionID
      // loop through list of states with their list of transitions
      allTransitions:
      for (Map.Entry<State, List<Transition>> exits : transitions.entrySet()) {

        for (Transition transition : exits.getValue()) {
          if (transition.transitionID == currentTransitionID) {
            exits.getKey().stateCommandAugmented.cancel();
            currentState = transition.nextState; // current state from previous state trigger that got us here
            break allTransitions;
          }
        }
      }
    }
    else {
      currentState = initialState; // nothing transitioned to here so must be initial state that was scheduled
    }
    // have current state so make triggers for all of its transitions
    var stateTransitions = transitions.get(currentState); // get the list of transitions for this State
    if (stateTransitions != null) { // if it is null then command will end (we are hopeful) with no exit transitions 
      for (Transition transition : stateTransitions) { //  add all the events for this state
        new Trigger (events, transition.triggeringEvent)
          .onTrue(transition.nextState.stateCommandAugmented);
      }
    }
    m_command.initialize(); // original command  
 }

 /**
  * The action to take when the command ends. Called when either the command finishes normally, or
  * when it interrupted/canceled.
  * @param interrupted whether the command was interrupted/canceled
  */
 public void end(boolean interrupted) {
  System.out.println("wrapper end interrupt " + interrupted);
   // find which event interrupted this state and schedule the next state
   m_command.end(interrupted);
 }

 /**
  * Whether the command has finished. Once a command finishes, the scheduler will call its end()
  * method and un-schedule it.
  *
  * @return whether the command has finished.
  */
 public boolean isFinished() {
    stateCompleted = m_command.isFinished(); // check for the command finished, remember that and pass that along
    if (stateCompleted) System.out.println("Wrapper isFinished " + stateCompleted);
    return stateCompleted;
 }
}

/////////////////////////////////////
// "ONE-TIME" SETUP THE STATE MACHINE
/////////////////////////////////////

  EventLoop events = new EventLoop();
  // each State will have 0 to many transitions added by the user
  Map<State, List<Transition>> transitions = new HashMap<State, List<Transition>>(); // All the States and their transitions of the FSM
  int transitionID = 0;
  int currentTransitionID = 0; // counter variable indicates the transition that was used to get to the new state; used to find the command the user specified for the new state
  
  private State initialState;
  
  boolean stateCompleted;

  public State addInitialState(Command initialStateCommand) { // save initial state command
    initialState = addState(initialStateCommand);
    return initialState;
  }

  public State addState(Command stateCommand) {
    return new State(stateCommand);
  }

  /**
   * class State as a command with transitions (event + next state command)
   */
  public class State extends Command
  {
    Command stateCommandAugmented; // the Wrapped (instrumented) state command that will actually be run
    List<Transition> exits = new ArrayList<Transition>(); // the transitions for this State

    /**
     * creating a new State from a command
     * @param stateCommand
     */
    private State(Command stateCommand) {
      this.stateCommandAugmented = new WrapState(stateCommand);
    }

    public void addTransition(State toNextState, BooleanSupplier whenEvent) { // imply run until the event then transition
      transitionID++;
      final int ID = transitionID;
      exits.add(new Transition(toNextState, ()-> {currentTransitionID = ID; return whenEvent.getAsBoolean();}, transitionID)); // add a transition to the list of this state
      transitions.put(this, exits); // list of transitions for this state in the master list of all state transitions
      new Trigger(whenEvent).onTrue(Commands.print("tripped the trigger"));
    }

    public void addTransition(State toNextState) { // imply run until completed then transition
      transitionID++;
      final int ID = transitionID;
      exits.add(new Transition(toNextState, ()-> {currentTransitionID = ID; return stateCompleted;}, transitionID)); // add a transition to the list of this state
      transitions.put(this, exits); // list of transitions for this state in the master list of all state transitions
    }
  } // end class State

  /**
   * class Transition is a Triggering EVENT to change to the next COMMAND (STATE)
   */
  private class Transition {
    State nextState; // also, essentially the "key" to finding a state
    BooleanSupplier triggeringEvent;
    int transitionID; // unique ID as the "key" to find a transition

    private Transition(State toNextState, BooleanSupplier whenEvent, int transitionID) {
      this.nextState = toNextState;
      this.triggeringEvent = whenEvent; 
      this.transitionID = transitionID;
    }
  } // end class Transition

  /**
   * Usage:
   * <pre><code>
    StateMachine.testit();
    </code></pre>
   * Change the state of Digital Input 0 (0, 1, 0, 1, 0, etc.) to indicate atScoringPosition and
   * state changes from pathing to scoring. Changes of states scoring and celebrating are automatic
   * when those states end when their functions are completed. 
   */
  public static void FSMtest() {
    System.out.println("StateMachine");

    final DigitalInput x = new DigitalInput(0);
    BooleanSupplier atScoringLocation = ()->x.get();

    Command drive_followPath = Commands.deadline(Commands.waitSeconds(1000.), Commands.print("pathing")).finallyDo(()->System.out.println("end pathing"));
    Command scorer_score = Commands.print("scoring").andThen(Commands.waitSeconds(1.)).finallyDo(()->System.out.println("end scoring"));
    Command leds_celebrate = Commands.print("celebrating").andThen(Commands.waitSeconds(1.)).finallyDo(()->System.out.println("end celebrating"));

    StateMachine auto = new StateMachine();

    State pathing = auto.addInitialState(drive_followPath);
    State scoring = auto.addState(scorer_score);
    State celebrating = auto.addState(leds_celebrate);

    pathing.addTransition(scoring, atScoringLocation);
    scoring.addTransition(celebrating);
    celebrating.addTransition(pathing);

    auto.schedule();
        /* testit results
        StateMachine
        pathing
        tripped the trigger
        wrapper end interrupt true
        end pathing
        scoring
        Wrapper isFinished true
        wrapper end interrupt false
        end scoring
        celebrating
        Wrapper isFinished true
        wrapper end interrupt false
        end celebrating
        */    
  }
} // end class StateMachine

/* Sam C. specification
// Declare the state machine
StateMachine auto = new StateMachine();

// Declare states
State pathing = auto.addInitialState(drive.followPath());
State scoring = auto.addState(scorer.score());
State celebrating = auto.addState(leds.celebrate());

// Declare transitions between states
pathing.transitionsTo(scoring).when(atScoringLocation);
scoring.transitionsTo(celebrating).whenComplete();

// StateMachine implements Command for seamless
// integration with the commands API
RobotModeTriggers.autonomous().onTrue(auto);
*/



// print transition map
// for (Map.Entry<State, List<Transition>> allExits : auto.transitions.entrySet()) {
//   System.out.println("FSM state  transitions " + allExits.getKey());
//   for (Transition transition : allExits.getValue()) {
//     System.out.println("transition " + transition + " , " + transition.nextState + " , " + transition.triggeringEvent + " , " + transition.transitionID);
//   }
// }
