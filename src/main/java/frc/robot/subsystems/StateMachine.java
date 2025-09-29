package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// @SuppressWarnings("unused")
/**
 * Example of a Finite State Machine (FSM) using simple methods to build the FSM from those utility
 * class. Based on the user-facing appearance of Command-Based V3 (as of 9/2025).
 * 
 * <p>Command-Based classes are used to wrap the users commands and triggers in order to define the
 * FSM "cyclic" behavior.
 * 
 * <p>This code has incomplete validation to prevent really bad parameters such as inappropriate nulls.
 * 
 * <p>This code has several print statements to show the execution of the state commands (normally
 * specified by the user) and also several print statements that are considered debugging of the
 * StateMachine flow (they should be removed for productive use).
 */
public class StateMachine extends Command {

  /////////////////////////////////////
  // "ONE-TIME" SETUP THE STATE MACHINE
  /////////////////////////////////////
  private String name;
  private class FSMsubsystem extends SubsystemBase {}
  private final SubsystemBase FSMrequirements = new FSMsubsystem();
  private boolean FSMfinished;
  private EventLoop events = new EventLoop();
  private List<State> states = new ArrayList<State>(); // All the states users instantiate (and STOP) only needed for printing the StateMachine - not in the logical flow
  private int transitionID = 0; // unique sequence number for debugging only needed for printing the StateMachine - not in the logical flow
  private State initialState; // user must call setInitialState or runtime fails with this null pointer
  private State completedNormally = null; // used for transition trigger whenComplete
  private final Command stopFSM = Commands.print("stopping FSM").andThen(Commands.runOnce(()->FSMfinished = true)); // mark FSM to stop to end that running command, too
  public final State stop = new State("stopped state", stopFSM);
  
  public StateMachine(String name) {
    this.name = name;
    stopFSM.setName("State Machine Stopped");
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
    System.out.println("StateMachine end interrupted " + interrupted);
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
      command.addRequirements(FSMrequirements); // lets scheduler cancel conflicting requirements - only one state may run at a time
    }

    /**
     * We're here because somebody scheduled us and we are running.
     * The initial state was scheduled when the StateMachine started.
     * All the rest of the states to run must be scheduled by an event triggering them onTrue.
     */
    @Override
    public void initialize() {
      events.clear(); // wipe the previous state's triggers
      System.out.println("state requiring FSMrequirements " + CommandScheduler.getInstance().requiring(FSMrequirements).getName());

      // make triggers for all of the current state's transitions
      if (state.transitions == null) {
        System.out.println("no transitions from state " + state.name);
      }
      else {
        for (Transition transition : state.transitions) { //  add all the events for this state
          new Trigger (events, transition.triggeringEvent)
            .onTrue(transition.nextState.stateCommandAugmented);
          System.out.println(state.name + " made exit trigger from " + transition.transitionID);
        }
      }

      completedNormally = null; // reset for this new state

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
      if (!interrupted) completedNormally = state; // indicate state ended by itself without others help
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
        System.out.println("Wrapper isFinished; everything completed normally " + completed);
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
        new Trigger(condition).onTrue(Commands.print("debug external tripped the trigger " + ID)); // debugging only
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
        new Trigger(condition).onTrue(Commands.print("debug check state completed tripped the trigger " + ID)); // debugging only
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
   * <pre><code>
    StateMachine.FSMtest();
    </code></pre>
   * Change the state of Digital Input 0 (0, 1, 0, 1, 0, etc.) to indicate atScoringPosition and
   * state changes from pathing to scoring. Changes of states scoring and celebrating are automatic
   * when those states end when their functions are completed. Celebrating can be forced to STOP
   * with the same Digital Input 0 (crude but easy and effective for a simple test method).
   */
  public static void FSMtest() {
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
    State testState = auto.addState("junk", Commands.none()); // testing unused state report
    
    auto.setInitialState(pathing);

    // any state that does not have exit transitions will be a "stop" state. This "auto.stop" is
    // provided as a convenience and is not required to be used.
    celebrating.switchTo(auto.stop).when(atScoringLocation);
    pathing.switchTo(scoring).when(atScoringLocation);
    scoring.switchTo(celebrating).whenComplete();
    celebrating.switchTo(pathing).whenComplete();
    
    auto.printStateMachine();

    auto.schedule();
  
  /* FSMtest results including CommandSchedulerLog to the console
  The DigitalInput 0 was manipulated to allow the states to cycle twice before activating the "stop" state.
  The Robot constructor was manipulated to slow the loop by a factor of 10 (less repetitive output)

StateMachine
All states for StateMachine Example State Machine
-------stopped state-------
Notice - State has no exits and if entered will stop the FSM.

-------pathing state-------
transition 2 frc.robot.subsystems.StateMachine$Transition@11423441 to scoring state with trigger frc.robot.subsystems.StateMachine$$Lambda$102/0x0000020e450a8200@3ccc2f3c

-------scoring state-------
transition 3 frc.robot.subsystems.StateMachine$Transition@3c94f42d to celebrating state with trigger frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$106/0x0000020e450a9f98@39294734

-------celebrating state-------
transition 1 frc.robot.subsystems.StateMachine$Transition@641bccb7 to stopped state with trigger frc.robot.subsystems.StateMachine$$Lambda$102/0x0000020e450a8200@3ccc2f3c
transition 4 frc.robot.subsystems.StateMachine$Transition@297849cf to pathing state with trigger frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$106/0x0000020e450a9f98@32d6e551

-------junk-------
Caution - State has no entrances and will not be used.

state requiring FSMrequirements PathCommand
pathing state made exit trigger from 2
 initializingPathCommand
Command initialized : WrapState/PathCommand {FSMsubsystem}
Command initialized : ScheduleCommand/ScheduleCommand {}
Command initialized : StateMachine/StateMachine {}
Command initialized : /StateMachine {}
Command initialized : InstantCommand/InstantCommand {}
Command executed : /StateMachine
Command finished : /StateMachine after 1 runs
Command executed : StateMachine/StateMachine
Command executed : ScheduleCommand/ScheduleCommand
Command finished : ScheduleCommand/ScheduleCommand after 1 runs
 execute PathCommandCommand executed : WrapState/PathCommand
Command executed : InstantCommand/InstantCommand
Command finished : InstantCommand/InstantCommand after 1 runs
 execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommanddebug external tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug external tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
 execute PathCommandCommand executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
 ending PathCommand interrupted true
wrapper end by interrupt true
WrapState/PathCommand after 32 runs interrupted by command WrapState/ScoreCommand
state requiring FSMrequirements ScoreCommand
scoring state made exit trigger from 3
 initializingScoreCommand
Command initialized : WrapState/ScoreCommand {FSMsubsystem}
 execute ScoreCommandCommand executed : WrapState/ScoreCommand
 execute ScoreCommand execute ScoreCommand execute ScoreCommand execute ScoreCommand execute ScoreCommandWrapper isFinished; everything completed normally true
 ending ScoreCommand interrupted false
wrapper end by interrupt false
Command finished : WrapState/ScoreCommand after 6 runs
debug check state completed tripped the trigger 3
Command initialized : PrintCommand/PrintCommand {}
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
state requiring FSMrequirements CelebrateCommand
celebrating state made exit trigger from 1
celebrating state made exit trigger from 4
 initializingCelebrateCommand
Command initialized : WrapState/CelebrateCommand {FSMsubsystem}
 execute CelebrateCommandCommand executed : WrapState/CelebrateCommand
 execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand
 execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommandWrapper isFinished; everything completed normally true
 ending CelebrateCommand interrupted false
wrapper end by interrupt false
Command finished : WrapState/CelebrateCommand after 101 runs
debug check state completed tripped the trigger 4
Command initialized : PrintCommand/PrintCommand {}
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
state requiring FSMrequirements PathCommand
pathing state made exit trigger from 2
 initializingPathCommand
Command initialized : WrapState/PathCommand {FSMsubsystem}
 execute PathCommandCommand executed : WrapState/PathCommand
 execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommanddebug external tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug external tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
 execute PathCommandCommand executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
 ending PathCommand interrupted true
wrapper end by interrupt true
WrapState/PathCommand after 28 runs interrupted by command WrapState/ScoreCommand
state requiring FSMrequirements ScoreCommand
scoring state made exit trigger from 3
 initializingScoreCommand
Command initialized : WrapState/ScoreCommand {FSMsubsystem}
 execute ScoreCommandCommand executed : WrapState/ScoreCommand
 execute ScoreCommand execute ScoreCommand execute ScoreCommand execute ScoreCommand execute ScoreCommandWrapper isFinished; everything completed normally true
 ending ScoreCommand interrupted false
wrapper end by interrupt false
Command finished : WrapState/ScoreCommand after 6 runs
debug check state completed tripped the trigger 3
Command initialized : PrintCommand/PrintCommand {}
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
state requiring FSMrequirements CelebrateCommand
celebrating state made exit trigger from 1
celebrating state made exit trigger from 4
 initializingCelebrateCommand
Command initialized : WrapState/CelebrateCommand {FSMsubsystem}
 execute CelebrateCommandCommand executed : WrapState/CelebrateCommand
 execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommanddebug external tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug external tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
 execute CelebrateCommandCommand executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
 ending CelebrateCommand interrupted true
wrapper end by interrupt true
WrapState/CelebrateCommand after 32 runs interrupted by command WrapState/SequentialCommandGroup
state requiring FSMrequirements SequentialCommandGroup
no transitions from state stopped state
stopping FSM
Command initialized : WrapState/SequentialCommandGroup {FSMsubsystem}
Command executed : WrapState/SequentialCommandGroup
StateMachine end interrupted false
Command finished : StateMachine/StateMachine after 210 runs
Wrapper isFinished; everything completed normally true
wrapper end by interrupt false
Command finished : WrapState/SequentialCommandGroup after 2 runs

*/
  } // end method FSMtest
} // end class StateMachine
