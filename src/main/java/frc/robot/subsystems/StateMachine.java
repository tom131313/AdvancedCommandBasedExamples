package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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

// @SuppressWarnings("unused") //FIXME remove
/**
 * Example of a Finite State Machine (FSM) using simple methods to build the FSM from those utility
 * class.
 * 
 * <p>This code has incomplete validation to prevent really bad parameters such as inappropriate nulls.
 * 
 * <p>Command-Based classes are used to wrap the users commands and triggers in order to define the
 * FSM "cyclic" behavior.
 */
public class StateMachine extends Command {

  /////////////////////////////////////
  // "ONE-TIME" SETUP THE STATE MACHINE
  /////////////////////////////////////

  private class FSMsubsystem extends SubsystemBase {}
  private final SubsystemBase FSMrequirements = new FSMsubsystem();
  private boolean FSMfinished;
  EventLoop events = new EventLoop();
  // each State will have 0 to many transitions added by the user
  Map<State, List<Transition>> transitions = new HashMap<State, List<Transition>>(); // All the States and their transitions of the FSM
  int transitionID = 0;
  int previousTransitionID = -1; // impossible value to force difference immediately
  private State initialState;
  private State completedNormally = null; // used for transition trigger whenComplete
  private Command stopFSM = Commands.print("stopping FSM").andThen(Commands.runOnce(()->FSMfinished = true)); // mark FSM to stop to end that running command, too
  public final State stop = new State("stopped state", stopFSM);
  public StateMachine(String name) {
    stopFSM.setName("State Machine Stopped");
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
  public void execute() {
    events.poll();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.

   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {
    System.out.println("StateMachine end interrupted " + interrupted);
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
    State state;
    
    WrapState(State state, Command command) {
      super(command); // should be state command to run at this time
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

      // make triggers for all of this current state's transitions
      var stateTransitions = transitions.get(state); // get the list of transitions for this State
      if (stateTransitions != null) { // if it is null then command will end (we are hopeful) with no exit transitions 
        for (Transition transition : stateTransitions) { //  add all the events for this state
          new Trigger (events, transition.triggeringEvent)
            .onTrue(transition.nextState.stateCommandAugmented);
          System.out.println(state.name + " made exit trigger from " + transition.transitionID);
        }
      }
      else {
        System.out.println("no transitions from state " + state.name);
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
      * Whether the command has finished. Once a command finishes, the scheduler will call its end()
      * method and un-schedule it.
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
    List<Transition> exits = new ArrayList<Transition>(); // the transitions for this State

    /**
     * creating a new State from a command
     * @param stateCommand
     */
    private State(String name, Command stateCommand) {
      this.name = name;
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
        exits.add(new Transition(m_targetState, condition, ID)); // wrap condition and add to the list a transition to this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
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
        exits.add(new Transition(m_targetState, condition, ID)); // wrap condition and add to the list a transition to this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
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
   * Because of the Digital Input resource is allocated herein without closing it, this
   * test case method can not be rerun without restarting the program. Normal usage would
   * be the triggering resources would be more independent of the StateMachine.
   * <pre><code>
    StateMachine.testit();
    </code></pre>
   * Change the state of Digital Input 0 (0, 1, 0, 1, 0, etc.) to indicate atScoringPosition and
   * state changes from pathing to scoring. Changes of states scoring and celebrating are automatic
   * when those states end when their functions are completed. 
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
    
    auto.setInitialState(pathing);

    // any state that does not have exit transitions will be a "stop" state. This "auto.stop" is
    // provided as a convenience and is not required to be used.
    celebrating.switchTo(auto.stop).when(atScoringLocation);
    pathing.switchTo(scoring).when(atScoringLocation);
    scoring.switchTo(celebrating).whenComplete();
    celebrating.switchTo(pathing).whenComplete();
    
    auto.printTransitions("");

    auto.schedule();
  
  /* FSMtest results
  The DigitalInput 0 was manipulated to allow the states to cycle twice before activating the "stop" state.

  Command initialized : InstantCommand/InstantCommand {}
StateMachine

 FSM state transitions from scoring state
transition frc.robot.subsystems.StateMachine$Transition@4e18a272 , to celebrating state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$110/0x000002188b0a9f80@36dd2e41 , ID 3

 FSM state transitions from celebrating state
transition frc.robot.subsystems.StateMachine$Transition@11632453 , to stopped state , frc.robot.subsystems.StateMachine$$Lambda$107/0x000002188b0a8400@2256814 , ID 1
transition frc.robot.subsystems.StateMachine$Transition@2a916681 , to pathing state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$110/0x000002188b0a9f80@2bfdf385 , ID 4   

 FSM state transitions from pathing state
transition frc.robot.subsystems.StateMachine$Transition@23340b15 , to scoring state , frc.robot.subsystems.StateMachine$$Lambda$107/0x000002188b0a8400@2256814 , ID 2
state requiring FSMrequirements PathCommand
pathing state made exit trigger from 2
 initializingPathCommand
Command initialized : WrapState/PathCommand {FSMsubsystem}
Command initialized : ScheduleCommand/ScheduleCommand {}
Command initialized : StateMachine/StateMachine {}
Command executed : InstantCommand/InstantCommand
Command finished : InstantCommand/InstantCommand after 1 runs
Command executed : StateMachine/StateMachine
Command executed : ScheduleCommand/ScheduleCommand
Command finished : ScheduleCommand/ScheduleCommand after 1 runs
 execute PathCommandCommand executed : WrapState/PathCommand
 execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommanddebug external tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug external tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
 execute PathCommandCommand executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
 ending PathCommand interrupted true
wrapper end by interrupt true
WrapState/PathCommand after 45 runs interrupted by command WrapState/ScoreCommand
state requiring FSMrequirements ScoreCommand
scoring state made exit trigger from 3
 initializingScoreCommand
Command initialized : WrapState/ScoreCommand {FSMsubsystem}
 execute ScoreCommandCommand executed : WrapState/ScoreCommand
 execute ScoreCommand execute ScoreCommand execute ScoreCommand execute ScoreCommand execute ScoreCommandWrapper isFinished; everything completed normally true
 ending ScoreCommand interrupted false
wrapper end by interrupt false
Command finished : WrapState/ScoreCommand after 6 runs
DataLog: Renamed log file from 'FRC_TBD_312fbfafc2b33d91.wpilog' to 'FRC_20250928_203615.wpilog'
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
 execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommandWrapper isFinished; everything completed normally true
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
 execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommand execute PathCommanddebug external tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug external tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
 execute PathCommandCommand executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
 ending PathCommand interrupted true
wrapper end by interrupt true
WrapState/PathCommand after 19 runs interrupted by command WrapState/ScoreCommand
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
 execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommand execute CelebrateCommanddebug external tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug external tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
 execute CelebrateCommandCommand executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
 ending CelebrateCommand interrupted true
wrapper end by interrupt true
WrapState/CelebrateCommand after 42 runs interrupted by command WrapState/SequentialCommandGroup
state requiring FSMrequirements SequentialCommandGroup
no transitions from state stopped state
stopping FSM
Command initialized : WrapState/SequentialCommandGroup {FSMsubsystem}
Command executed : WrapState/SequentialCommandGroup
StateMachine end interrupted false
Command finished : StateMachine/StateMachine after 224 runs
Wrapper isFinished; everything completed normally true
wrapper end by interrupt false
Command finished : WrapState/SequentialCommandGroup after 2 runs
*/    
  } // end method FSMtest
} // end class StateMachine
