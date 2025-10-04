package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// @SuppressWarnings("unused")
/**
 * Example of a Finite State Machine (FSM) using simple methods to build the FSM from those utility
 * class. Based on the user-facing appearance of Command-Based V3 (as of 9/2025).
 * <p>This is essentially the same as typical coding of Triggers with conditions and onTrue commands.
 * The benefit of this FSM implementation is not so much changing the names of the two methods but
 * the state changing triggers exist only for the duration of the state instead of being a perpetual
 * part of the huge mass of triggers for the robot code.
 * 
 * <p>Command-Based classes are used to wrap the users commands and triggers in order to define the
 * FSM "cyclic" behavior.
 * 
 * <p>This code has incomplete validation to prevent really bad parameters such as inappropriate nulls.
 * 
 * <p>This code has several print statements to show the execution of the state commands (normally
 * specified by the user) and also several print statements that are considered debugging of the
 * StateMachine flow (they should be removed for productive use and are controlled herein with the
 * "debug" variable).
 */
public class StateMachine extends Command {

  /////////////////////////////////////
  // "ONE-TIME" SETUP THE STATE MACHINE
  /////////////////////////////////////
  
  private final boolean debug = false; //FIXME activate debug prints to terminal

  private String name;
  private boolean FSMfinished;
  private EventLoop events = new EventLoop();
  private List<State> states = new ArrayList<State>(); // All the states users instantiate (and STOP) only needed for printing the StateMachine - not in the logical flow
  private int transitionID = 0; // unique sequence number for debugging only needed for printing the StateMachine - not in the logical flow
  private State initialState; // user must call setInitialState or runtime fails with this null pointer
  private State completedNormally = null; // used for transition trigger whenComplete
  public final State stop;
  private Command stateCommandAugmentedPrevious = null; // need to know if previous is still running so can be cancelled on state transition

  public StateMachine(String name) {
    this.name = name;
    stop = addState("stopped state", new Stopper());
  }

  /**
   * Required to set the initial state or runtime fails with a null pointer.
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
   * Print transition information about the StateMachine
   */
  public void printStateMachine() {
    System.out.println("All states for StateMachine " + name);
    
    for (State state : states) {
      boolean noExits = true; // initially haven't found any
      boolean noEntrances = true; // initially haven't found any

      System.out.println("-------" + state.name + "-------");

      // loop through all the transitions of this state
      for (Transition transition : state.transitions) {
        noExits = false; // at least one transition out of this state
        System.out.println("transition " + transition.transitionID + " "
        + transition + " to " + transition.nextState.name + " with trigger " + transition.triggeringEvent);
      }          

      // loop through all the states again to find at least one entrance to this state
      allStates:
      for (State stateInner : states) {
        for (Transition transition : stateInner.transitions) {
          if (transition.nextState == state) {
            noEntrances = false;
            break allStates;
          }
        }
      }
  System.out.println(
    (noEntrances?"Caution - State has no entrances and will not be used.\n":
    noExits?"Notice - State has no exits and if entered will stop the FSM.\n":""));
  }
}

  /////////////////////////////////////////////////////
  // THE ITERATIVE CONTROL COMMAND OF THE STATE MACHINE
  /////////////////////////////////////////////////////
 
  /** The initial subroutine of a command. Called once when the command is scheduled. */
  public void initialize() {
    // System.out.println("StateMachine initialize");
    FSMfinished = false;
    events.clear(); // make sure clear in case there would be a race between the execute poll and the next command clear (maybe used if FSM can start/stop which it can't right now)
    new ScheduleCommand(initialState.stateCommandAugmented).schedule();
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  @Override
  public void execute() {
    events.poll(); // check for events that can trigger transitions out of this state
  }

  /**
   * This method overrides the super just so it can print the message for debugging.
   * If the message is no longer needed, then this entire method should be removed.
   * 
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.

   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    if (debug) System.out.println("StateMachine end interrupted " + interrupted);
    // the StateMachine manager is stopping so cancel the State command if it's still running
    if (stateCommandAugmentedPrevious != null) {
      stateCommandAugmentedPrevious.cancel();
    }
  }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  @Override
  public boolean isFinished() {
    return FSMfinished;
  }

  /////////////////////////////////////
  // RUN THE WRAPPED STATES AS COMMANDS
  /////////////////////////////////////
  /**
   * Wrap a command to define the state.
   * <p>The wrapper creates the event triggers that will change to the next state
   * and remember if the state command ended normally without interruption.
   */
  private class WrapState extends WrapperCommand {
    State state;
    
    WrapState(State state, Command command) {
      super(command); // user's original state command to run
      this.state = state;
    }

    /**
     * We're here because somebody scheduled us and we are running.
     * The initial state was scheduled when the StateMachine started.
     * All the rest of the states to run must be scheduled by an event triggering them onTrue.
     */
    @Override
    public void initialize() {
      events.clear(); // wipe the previous state's triggers
      if(stateCommandAugmentedPrevious != null) {
        stateCommandAugmentedPrevious.cancel(); // wipe the previous state in case it didn't finish itself
      }
       // make triggers for all of the current state's transitions
      if (state.transitions == null) {
        if(debug) System.out.println("no transitions from state " + state.name);
      }
      else {
        for (Transition transition : state.transitions) { //  add all the events for this state
          new Trigger (events, transition.triggeringEvent)
            .onTrue(transition.nextState.stateCommandAugmented);
          if(debug) System.out.println(state.name + " made exit trigger from " + transition.transitionID);
        }
      }

      completedNormally = null; // reset for this new state
      stateCommandAugmentedPrevious = this;

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
      if(debug) System.out.println("wrapper end by interrupt " + interrupted);
      if (!interrupted) {
        completedNormally = state; // indicate state ended by itself without others help
      }
      stateCommandAugmentedPrevious = null; // this command completed so there is no effective previous to cancel
    }

    /**
     * This method overrides the super just so it can print the message for debugging.
     * If the message is no longer needed, then this entire method should be removed.
     * 
     * Once a command finishes, the scheduler will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
      var completed = m_command.isFinished(); // check original command finished by itself or not, remember that and pass that along
      if (completed) {
        if(debug) System.out.println("Wrapper isFinished; everything completed normally " + completed);
      }
      return completed; // Wrapper follows underlying command
    }
  } // end class WrapState

  /**
   * class State as a command with transitions (event + next state command)
   */
  public class State extends Command
  {
    private final String name;
    Command stateCommandAugmented; // the Wrapped (instrumented) state command that will actually be run
    List<Transition> transitions = new ArrayList<Transition>(); // the transitions for this State

    /**
     * creating a new State from a command
     * @param stateCommand
     */
    private State(String name, Command stateCommand) {
      this.name = name;
      StateMachine.this.states.add(this);
      this.stateCommandAugmented = new WrapState(this, stateCommand);
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
      // private final State m_originatingState;
      private final State m_targetState;

      private NeedsConditionTransitionBuilder(State from, State to) {
        // m_originatingState = from;
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
        transitions.add(new Transition(m_targetState, condition, ID)); // wrap condition and add to the list a transition to this state
        if(debug) new Trigger(condition).onTrue(Commands.print("debug external tripped the trigger " + ID));
      }

      /**
       * Marks the transition when the originating state completes without having reached any other
       * transitions first.
       */
      public void whenComplete() {
        BooleanSupplier condition = ()-> State.this == completedNormally;
        transitionID++;
        final int ID = transitionID;
        transitions.add(new Transition(m_targetState, condition, ID)); // wrap condition and add to the list a transition to this state
        if(debug) new Trigger(condition).onTrue(Commands.print("debug check state completed tripped the trigger " + ID));
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
   * Stop the StateMachine Command that does nothing except help define a stopped state. (The user
   * may provide any state desired that has no exits and the command ends normally to be the stop
   * state.)
   */
  private class Stopper extends Command {
    private Stopper() {
      setName("State Machine Stopper");
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }

    @Override
    public void initialize() {
      FSMfinished = true;
    }
    @Override
    public boolean isFinished() {
      return true;
    }
  }

  /**
   * Command Factory for testing StateMachine
   * 
   * Using this syntax is slightly easier to read than using "new TestDuration()".
   *
   * @param testName output this number and the resource (subsystem) ID
   * @param testDuration elapsed time to run execute() to produce output
   * @return command that puts out a "testNumber" for "testDuration" time
   */
  public Command testDuration(String testName, Time testDuration) {
    return new TestDuration(testName, testDuration);
  }

  private class TestDuration extends Command {
    private final String testName;
    private final Time testDuration;
    Timer endTime = new Timer();

    private TestDuration(String testName, Time testDuration) {
      this.testName = testName;
      this.testDuration = testDuration;
      setName(testName);
    }

    @Override
    public void initialize() {
      System.out.println(" initializing" + testName);
      endTime.restart();
    }

    @Override
    public void execute() {
      System.out.print(" execute " + testName);
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println(" ending " + testName + " interrupted " + interrupted);
    }

    @Override
    public boolean isFinished() {
      return endTime.hasElapsed(testDuration.in(Seconds));
    }
  }

  /**
   * Usage:
   * <p>Because of the Digital Input resource is allocated herein without closing it, this
   * test case method can not be rescheduled without restarting the program. Normal usage would
   * be the triggering resources would be more independent of the StateMachine.
   * <p>The Robot constructor "super(loop speed)" can be manipulated to slow the loop by a factor
   * of 10 or so for less repetitive output.
   * <pre><code>
    StateMachine.FSMtest();
    </code></pre>
   * Change the state of Digital Input 0 (0, 1, 0, 1, 0, etc.) to indicate atScoringPosition and
   * state changes from pathing to scoring. Changes of states scoring and celebrating are automatic
   * when those states end when their functions are completed. Celebrating can be forced to STOP
   * with the same Digital Input 0 (crude but easy and effective for a simple test method).
   * 
   * @throws edu.wpi.first.hal.util.AllocationException if run twice
   */
  public static void FSMtest() {
    try {
    System.out.println("StateMachine");
    @SuppressWarnings("resource")
    DigitalInput DI_0 = new DigitalInput(0);
    BooleanSupplier atScoringLocation = ()-> DI_0.get();

    StateMachine auto = new StateMachine("Example State Machine");

    Command drive_followPath = auto.testDuration("PathCommand", Seconds.of(1000.));
    Command scorer_score = auto.testDuration("ScoreCommand", Seconds.of(1.));
    Command leds_celebrate = auto.testDuration("CelebrateCommand", Seconds.of(20.));

    State pathing = auto.addState("pathing state", drive_followPath);
    State scoring = auto.addState("scoring state", scorer_score);
    State celebrating = auto.addState("celebrating state", leds_celebrate);
    State testState = auto.addState("junk", Commands.none()); // purposeful testing unused state report
    
    auto.setInitialState(pathing);

    // Any state that does not have an exit transition is a sort of "stop" state. This "auto.stop"
    // is provided as a convenience and will stop both itself and stop the StateMachine command.
    celebrating.switchTo(auto.stop).when(atScoringLocation); // 
    pathing.switchTo(scoring).when(atScoringLocation);
    scoring.switchTo(celebrating).whenComplete();
    celebrating.switchTo(pathing).whenComplete();
    
    auto.printStateMachine();

    auto.schedule();
    } catch(Exception e){System.out.println("StateMachine test method FMStest failed probably because it was invoked twice and DIO 0 wasn't closed (on purpose)." + e.getMessage());}
  } // end method FSMtest
} // end class StateMachine
