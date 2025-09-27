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
    System.out.println("Wrapper initialize");
    events.clear(); // wipe the previous state's triggers

      State currentState = null;
      if (currentTransitionID != 0) {
      // find the immediately preceding transition that got us here using currentTransitionID
      // loop through list of states with their list of transitions
      allTransitions:
      for (Map.Entry<State, List<Transition>> exitsIterate : transitions.entrySet()) {
        for (Transition transition : exitsIterate.getValue()) {
          if (transition.transitionID == currentTransitionID) {
            if (! stateCompletedNormally) exitsIterate.getKey().stateCommandAugmented.cancel(); // cancel previous since it didn't end of its own accord
            currentState = transition.nextState; // current state from previous state trigger that got us here
            System.out.println("cancelled " + exitsIterate.getKey().name + " entering " + currentState.name);
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
    
    stateCompletedNormally = false;

    m_command.initialize(); // Wrapper is done with its fussing so tell original command to initialize
 }

 /**
  * The action to take when the command ends. Called when either the command finishes normally, or
  * when it interrupted/canceled.
  * @param interrupted whether the command was interrupted/canceled
  */
  @Override
 public void end(boolean interrupted) {
   m_command.end(interrupted); // tell original command to end and if this wrapper was interrupted or not
   System.out.println("wrapper end by interrupt " + interrupted);
   if (!interrupted) stateCompletedNormally = true; // state ended by itself so this can be used for a whenComplete condition for trigger of the next state
 }

 /**
  * Whether the command has finished. Once a command finishes, the scheduler will call its end()
  * method and un-schedule it.
  *
  * @return whether the command has finished.
  */
 public boolean isFinished() {
    var completed = m_command.isFinished(); // check original command finished by itself or not, remember that and pass that along
    if (completed) System.out.println("Wrapper isFinished; everything completed normally " + completed);
    return completed; // Wrapper follows underlying command
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
  
  boolean stateCompletedNormally; // used for transition trigger whenComplete

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

  public State addState(String name, Command stateCommand) {
    return new State(name, stateCommand);
  }

  /**
   * class State as a command with transitions (event + next state command)
   */
  public class State extends Command
  {
    final String name;
    Command stateCommandAugmented; // the Wrapped (instrumented) state command that will actually be run
    List<Transition> exits = new ArrayList<Transition>(); // the transitions for this State

    /**
     * creating a new State from a command
     * @param stateCommand
     */
    private State(String name, Command stateCommand) {
      this.name = name;
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

      /**
       * Adds a transition that will be triggered when the specified condition is true.
       *
       * @param condition The condition that will trigger the transition.
       */
      public void when(BooleanSupplier condition) { // imply run until the event then transition
        transitionID++;
        final int ID = transitionID;
        exits.add(new Transition(m_targetState, ()-> {currentTransitionID = ID; return condition.getAsBoolean();}, transitionID)); // add a transition to the list of this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
        new Trigger(condition).onTrue(Commands.print("double check condition tripped the trigger"));
        // printTransitions("when with " + transitionID);
      }

      /**
       * Marks the transition when the originating state completes without having reached any other
       * transitions first.
       */
      public void whenComplete() {
        transitionID++;
        final int ID = transitionID;
        exits.add(new Transition(m_targetState, ()-> {currentTransitionID = ID; return stateCompletedNormally;}, transitionID)); // add a transition to the list of this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
        new Trigger(()->stateCompletedNormally).onTrue(Commands.print("double check state completed tripped the trigger"));
        // printTransitions("whenComplete with " + transitionID);
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

    /**
     * print transition map
     */
    public void printTransitions(String title) {
      for (Map.Entry<State, List<Transition>> allExits : transitions.entrySet()) {
        System.out.println("\n" + title + " FSM state transitions from " + allExits.getKey().name);
        for (Transition transition : allExits.getValue()) {
          System.out.println("transition "
           + transition + " , to " + transition.nextState.name + " , " + transition.triggeringEvent + " , ID " + transition.transitionID);
        }
      }
    }

    private Command stopFSM = Commands.runOnce(()->FSMfinished = true); // mark FSM to stop to end that running command, too
    public final State stop = new State("stop", stopFSM); // 

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

    final DigitalInput DI_0 = new DigitalInput(0);
    BooleanSupplier atScoringLocation = ()-> DI_0.get();

    Command drive_followPath = Commands.deadline(Commands.waitSeconds(1000.), Commands.print("pathing")).finallyDo(()->System.out.println("end pathing"));
    Command scorer_score = Commands.print("scoring").andThen(Commands.waitSeconds(1.)).finallyDo(()->System.out.println("end scoring"));
    Command leds_celebrate = Commands.print("celebrating").andThen(Commands.waitSeconds(19.)).finallyDo(()->System.out.println("end celebrating"));

    StateMachine auto = new StateMachine("Example State Machine");

    State pathing = auto.addState("pathing state", drive_followPath);
    State scoring = auto.addState("scoring state", scorer_score);
    State celebrating = auto.addState("celebrating state", leds_celebrate);
    
    auto.setInitialState(pathing);

    pathing.switchTo(scoring).when(atScoringLocation);
    scoring.switchTo(celebrating).whenComplete();
    celebrating.switchTo(pathing).whenComplete();
    celebrating.switchTo(auto.stop).when(atScoringLocation);
    auto.printTransitions("");

    auto.schedule();
        /* FSMtest results
refactor chained build work in progress not right yet
 - note that the test case was changed so results should be different but t his isn't right, I think.

StateMachine

 FSM state transitions from scoring state
transition frc.robot.subsystems.StateMachine$Transition@476f1571 , to celebrating state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$186/0x000001a7810c0428@51d7deab , ID 2

 FSM state transitions from pathing state
transition frc.robot.subsystems.StateMachine$Transition@79c14819 , to scoring state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$185/0x000001a7810c0208@b025376 , ID 1

 FSM state transitions from celebrating state
transition frc.robot.subsystems.StateMachine$Transition@7166318c , to pathing state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$186/0x000001a7810c0428@5bb32eef , ID 3
transition frc.robot.subsystems.StateMachine$Transition@28c12400 , to stop , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$185/0x000001a7810c0208@31c14b8d , ID 4
Wrapper initialize

pathing
double check condition tripped the trigger
double check condition tripped the trigger
Wrapper initialize
end pathing
wrapper end by interrupt true

cancelled pathing state entering scoring state

scoring
Wrapper isFinished; everything completed normally true
end scoring
wrapper end by interrupt false

double check state completed tripped the trigger
double check state completed tripped the trigger

Wrapper initialize

cancelled scoring state entering celebrating state
celebrating
Wrapper isFinished; everything completed normally true
end celebrating
wrapper end by interrupt false

double check state completed tripped the trigger
double check state completed tripped the trigger

Wrapper initialize
cancelled celebrating state entering stop
pathing
---hung-- ???????
double check condition tripped the trigger
double check condition tripped the trigger






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
