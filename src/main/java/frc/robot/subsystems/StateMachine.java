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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  int currentTransitionID = 0; // initially counter-type variable indicates the transition that was used to get to the new state; later used to find the command the user specified for the new state
  int currentTransitionIDPrevious = -1; // impossible value to force difference immediately
  private State initialState;
  boolean previousStateCompletedNormally; // used for transition trigger whenComplete and cancelling running states
  private Command stopFSM = Commands.print("stopping FSM").andThen(Commands.runOnce(()->FSMfinished = true)); // mark FSM to stop to end that running command, too
  public final State stop = new State("stopped state", stopFSM);
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
    events.clear(); // make sure clear in case there would be a race between the execute poll and the next command clear
    new ScheduleCommand(initialState.stateCommandAugmented).schedule();
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  public void execute() {
    // System.out.println("StateMachine execute");
    if (currentTransitionID != currentTransitionIDPrevious) { // debug print control
      System.out.println("previous ID " + currentTransitionIDPrevious + " current ID " + currentTransitionID);
      currentTransitionIDPrevious = currentTransitionID;
    }

    events.poll();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.

   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {
    System.out.println("StateMachine end; interrupted " + interrupted);
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

    @Override
    public void initialize() {
      System.out.println("Arrived via transition " + currentTransitionID + " Wrapper initializing");
      events.clear(); // wipe the previous state's triggers

        State currentState = null;
        if (currentTransitionID != 0) {
        // find the immediately preceding transition that got us here using currentTransitionID
        // loop through list of states with their list of transitions
        allTransitions:
        for (Map.Entry<State, List<Transition>> exitsIterate : transitions.entrySet()) {
          for (Transition transition : exitsIterate.getValue()) {
            if (transition.transitionID == currentTransitionID) { // see if this is the transition that got us here
              // if ( ! previousStateCompletedNormally) exitsIterate.getKey().stateCommandAugmented.cancel(); // cancel previous since it hasn't ended of its own accord
              // if (exitsIterate.getKey().stateCommandAugmented.isScheduled()) exitsIterate.getKey().stateCommandAugmented.cancel(); // cancel previous since it hasn't ended of its own accord
              currentState = transition.nextState; // current state from previous state trigger that got us here
              System.out.println("exiting " + exitsIterate.getKey().name + " now entering " + currentState.name);
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
          System.out.println(currentState.name + " made exit trigger from " + transition.transitionID);
        }
      }
      else {
        System.out.println("no transitions from state " + currentState.name);
      }
      
      previousStateCompletedNormally = false; // reset for this new state

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
    if (!interrupted) previousStateCompletedNormally = true; // indicate state ended by itself without others help
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
      if (completed) System.out.println("Wrapper isFinished; everything completed normally " + completed);
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
      this.stateCommandAugmented = new WrapState(stateCommand);
      this.stateCommandAugmented.addRequirements(FSMrequirements);
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
        exits.add(new Transition(m_targetState, ()-> {currentTransitionID = ID; if(condition.getAsBoolean()) System.out.println("tripped " + currentTransitionID);
           return condition.getAsBoolean();}, transitionID)); // wrap condition and add to the list a transition to this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
        new Trigger(condition).onTrue(Commands.print("debug check condition tripped the trigger " + ID));
        // printTransitions("when with " + transitionID);
      }

      /**
       * Marks the transition when the originating state completes without having reached any other
       * transitions first.
       */
      public void whenComplete() {
        transitionID++;
        final int ID = transitionID;
        exits.add(new Transition(m_targetState, ()-> {currentTransitionID = ID; if(previousStateCompletedNormally) System.out.println("tripped " + currentTransitionID);
           return previousStateCompletedNormally;}, transitionID)); // wrap condition and add to the list a transition to this state
        transitions.put(State.this, exits); // list of transitions for this state in the master list of all state transitions
        new Trigger(()->previousStateCompletedNormally).onTrue(Commands.print("debug check state completed tripped the trigger " + ID));
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

    Command drive_followPath = Commands.deadline(Commands.waitSeconds(1000.), Commands.print("pathing")).finallyDo(()->System.out.println("finallyDo end pathing"));
    drive_followPath.setName("pathCommand");
    Command scorer_score = Commands.print("scoring").andThen(Commands.waitSeconds(1.)).finallyDo(()->System.out.println("finallyDo end scoring"));
    scorer_score.setName("scoreCommand");
    Command leds_celebrate = Commands.print("celebrating").andThen(Commands.waitSeconds(20.)).finallyDo(()->System.out.println("finallyDo end celebrating"));
    leds_celebrate.setName("celebrateCommand");

    StateMachine auto = new StateMachine("Example State Machine");

    State pathing = auto.addState("pathing state", drive_followPath);
    State scoring = auto.addState("scoring state", scorer_score);
    State celebrating = auto.addState("celebrating state", leds_celebrate);
    
    auto.setInitialState(pathing);

    celebrating.switchTo(auto.stop).when(atScoringLocation); // 1
    pathing.switchTo(scoring).when(atScoringLocation); // 2
    scoring.switchTo(celebrating).whenComplete(); // 3
    celebrating.switchTo(pathing).whenComplete(); // 4
// 0 2 3 1
    auto.printTransitions("");

    auto.schedule();
  
  /* FSMtest results
Command initialized : InstantCommand/InstantCommand {}
StateMachine

 FSM state transitions from scoring state
transition frc.robot.subsystems.StateMachine$Transition@140a8141 , to celebrating state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$115/0x0000016e810aa808@7b3aa352 , ID 3

StateMachine
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:436): Loop time of 0.02s overrun


 FSM state transitions from celebrating state
transition frc.robot.subsystems.StateMachine$Transition@7b3aa352 , to stopped state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$113/0x0000011b9d0adbb8@3bb3e820 , ID 1
transition frc.robot.subsystems.StateMachine$Transition@7ad6e43a , to pathing state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$115/0x0000011b9d0ae418@75cd09b1 , ID 4

 FSM state transitions from pathing state
transition frc.robot.subsystems.StateMachine$Transition@5086a67c , to scoring state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$113/0x0000011b9d0adbb8@d6c6111 , ID 2

 FSM state transitions from scoring state
transition frc.robot.subsystems.StateMachine$Transition@4056b3b2 , to celebrating state , frc.robot.subsystems.StateMachine$State$NeedsConditionTransitionBuilder$$Lambda$115/0x0000011b9d0ae418@5d4bba88 , ID 3
Arrived via transition 0 Wrapper initializing
tripped 2
pathing state made exit trigger from 2
pathing
Command initialized : WrapState/pathCommand {}
Command initialized : ScheduleCommand/ScheduleCommand {}
Command initialized : StateMachine/Example State Machine {}
Command executed : InstantCommand/InstantCommand
Command finished : InstantCommand/InstantCommand after 1 runs
previous ID -1 current ID 2
tripped 2
Command executed : StateMachine/Example State Machine
Command executed : ScheduleCommand/ScheduleCommand
Command finished : ScheduleCommand/ScheduleCommand after 1 runs
Command executed : WrapState/pathCommand
tripped 2
tripped 2

tripped 2
tripped 2
debug check condition tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug check condition tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
tripped 2
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Arrived via transition 2 Wrapper initializing
exiting pathing state now entering scoring state
scoring state made exit trigger from 3
scoring
Command initialized : WrapState/scoreCommand {}
previous ID 2 current ID 3
Command executed : WrapState/scoreCommand
Wrapper isFinished; everything completed normally true
finallyDo end scoring
wrapper end by interrupt false
Command finished : WrapState/scoreCommand after 52 runs
debug check state completed tripped the trigger 3
Command initialized : PrintCommand/PrintCommand {}
debug check state completed tripped the trigger 4
Command initialized : PrintCommand/PrintCommand {}
tripped 3
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Arrived via transition 3 Wrapper initializing
exiting scoring state now entering celebrating state
tripped 1
celebrating state made exit trigger from 1
tripped 4
celebrating state made exit trigger from 4
celebrating
Command initialized : WrapState/celebrateCommand {}
previous ID 3 current ID 4
tripped 1
Command executed : WrapState/celebrateCommand
tripped 1
tripped 1

tripped 1
tripped 1
debug check condition tripped the trigger 1
Command initialized : PrintCommand/PrintCommand {}
debug check condition tripped the trigger 2
Command initialized : PrintCommand/PrintCommand {}
tripped 1
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Arrived via transition 4 Wrapper initializing
exiting celebrating state now entering pathing state
tripped 2
pathing state made exit trigger from 2
stopping FSM
Command initialized : WrapState/SequentialCommandGroup {}
previous ID 4 current ID 2
tripped 2
Command executed : WrapState/SequentialCommandGroup
tripped 2
StateMachine end; interrupted false
Command finished : StateMachine/Example State Machine after 618 runs
Wrapper isFinished; everything completed normally true
wrapper end by interrupt false
Command finished : WrapState/SequentialCommandGroup after 2 runs
debug check state completed tripped the trigger 3
Command initialized : PrintCommand/PrintCommand {}
debug check state completed tripped the trigger 4
Command initialized : PrintCommand/PrintCommand {}
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Command executed : PrintCommand/PrintCommand
Command finished : PrintCommand/PrintCommand after 1 runs
Wrapper isFinished; everything completed normally true
finallyDo end celebrating
wrapper end by interrupt false
Command finished : WrapState/celebrateCommand after 1002 runs
*/    
  } // end method FSMtest
} // end class StateMachine
