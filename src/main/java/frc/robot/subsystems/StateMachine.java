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
  
  private boolean FSMfinished;
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
      for (Map.Entry<State, List<Transition>> exitsIterate : transitions.entrySet()) {

        for (Transition transition : exitsIterate.getValue()) {
          if (transition.transitionID == currentTransitionID) {
            exitsIterate.getKey().stateCommandAugmented.cancel();
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
} // end class WrapState

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

  public StateMachine(String name) {
    setName(name);
  }

    /**
   * Sets the initial state for the state machine.
   *
   * @param initialState The new initial state. Cannot be null.
   */
  public void setInitialState(State initialState) {
    this.initialState = initialState;
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

    /**
     * Starts building a transition to the specified state.
     *
     * @param to The state to transition to. Cannot be null.
     * @return A builder for the transition.
     */
    public NeedsConditionTransitionBuilder switchTo(State to) {
      return new NeedsTargetTransitionBuilder(this).to(to);
    }

    /**
     * A builder for a transition from one state to another. Use {@link #to(State)} to specify the
     * target state to transition to.
     */
    public final class NeedsTargetTransitionBuilder {
      private final State m_from;

      private NeedsTargetTransitionBuilder(State from) {
        m_from = from;
      }

      /**
       * Specifies the target state to transition to.
       *
       * @param to The state to transition to. Cannot be null.
       * @return A builder to specify the transition condition.
       */
      private NeedsConditionTransitionBuilder to(State to) {
        return new NeedsConditionTransitionBuilder(m_from, to);
      }
    } // end class NeedsTargetTransitionBuilder

    /**
     * A builder to set conditions for a transition from one state to another. Use {@link
     * #when(BooleanSupplier)} to make the transition occur when some external condition becomes true,
     * or use {@link #whenComplete()} to make the transition occur when the originating state
     * completes without having reached any other transitions first.
     */
    public final class NeedsConditionTransitionBuilder {
      private final State m_originatingState;
      private final State m_targetState;

      private NeedsConditionTransitionBuilder(State from, State to) {
        m_originatingState = from;
        m_targetState = to;
      }

      // /**
      //  * Adds a transition that will be triggered when the specified condition is true.
      //  *
      //  * @param condition The condition that will trigger the transition. Cannot be null.
      //  */
      // public void when(BooleanSupplier condition) {
      //   m_originatingState.addTransition(new Transition(m_targetState, condition));
      // }

      // /**
      //  * Marks the transition when the originating state completes without having reached any other
      //  * transitions first.
      //  */
      // public void whenComplete() {
      //   m_originatingState.setNextState(m_targetState);
      // }

      // like when
      public void when/*switchTo*/(BooleanSupplier condition) { // imply run until the event then transition
        transitionID++;
        final int ID = transitionID;
        exits.add(new Transition(m_targetState, ()-> {currentTransitionID = ID; return condition.getAsBoolean();}, transitionID)); // add a transition to the list of this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
        new Trigger(condition).onTrue(Commands.print("tripped the trigger"));
      }

      // like whenComplete
      public void whenComplete/*switchTo*/() { // imply run until completed then transition
        transitionID++;
        final int ID = transitionID;
        exits.add(new Transition(m_targetState, ()-> {currentTransitionID = ID; return stateCompleted;}, transitionID)); // add a transition to the list of this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
      }
    } // end class NeedsConditionTransitionBuilder
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

    private Command stopFSM = Commands.runOnce(()->FSMfinished = true); // mark FSM to stop to end that running command, too
    public final State stop = new State(stopFSM); // 

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
    Command leds_celebrate = Commands.print("celebrating").andThen(Commands.waitSeconds(19.)).finallyDo(()->System.out.println("end celebrating"));

    StateMachine auto = new StateMachine("Example State Machine");

    State pathing = auto.addState(drive_followPath);
    State scoring = auto.addState(scorer_score);
    State celebrating = auto.addState(leds_celebrate);
    auto.setInitialState(pathing);

    pathing.switchTo(scoring).when(atScoringLocation);
    scoring.switchTo(celebrating).whenComplete();
    celebrating.switchTo(pathing).whenComplete();
    celebrating.switchTo(auto.stop).when(atScoringLocation);

    auto.schedule();
        /* testit results
refactor chained build work in progress not right yet
 - note that the test case was changed so results should be different but t his isn't right, I think.
StateMachine
pathing
tripped the trigger
tripped the trigger
wrapper end interrupt true
end pathing
scoring
Wrapper isFinished true
wrapper end interrupt false
end scoring
celebrating
tripped the trigger
tripped the trigger
wrapper end interrupt true
end celebrating
Wrapper isFinished true
wrapper end interrupt false
tripped the trigger
tripped the trigger





previous good
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
  } // end method FSMtest
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
