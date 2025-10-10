package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// @SuppressWarnings("unused")
/**
 * Example of a Finite State Machine (FSM) using simple methods to build the FSM.
 * 
 * <p>Based on the user-facing appearance of Command-Based V3 (as of 10/2025).
 * 
 * <p>This is essentially the same as typical coding of Triggers with conditions and onTrue commands.
 * The benefit of this FSM implementation is not so much changing the names of the methods but the
 * state changing triggers exist only for the duration of the state instead of being a perpetual
 * part of the huge mass of triggers for the robot code.
 * 
 * <p>Another feature is an automatically created
 * internal trigger for when a state command completes normally instead of being interrupted. Use
 * "whenComplete()" to use that feature. Use "when()" for a typical external trigger condition.
 * 
 * <p>Command-Based classes are used to wrap the users commands and triggers in order to define the
 * FSM "cyclic" behavior.
 * 
 * <p>This code has incomplete validation to prevent really bad parameters such as inappropriate nulls.
 * 
 * <p>Any state without an exit transition is a stop state if entered and completes. Example stop state
 * shown below. This ends the command running the StateMachine.
 * 
 * <p>The StateMachine does not have an idle state. Any state entered and does nothing until interrupted
 * would be idle for its duration. Example idle state shown below could be used to keep the StateMachine
 * running so it does not end and would not need to be recreated for a restart.
 * 
 *<pre><code>
 * {@literal /}**
 *  * Example factory of an example state machine
 *  * 
 *  * {@literal @@return} state machine that must be scheduled in some manner -
 *  * .schedule(), triggered by a condition or button press, for example.
 *  *{@literal /}
 * public StateMachine createStateMachine()
 * {
 *       // first you need a StateMachine
 *       var stateMachine = new StateMachine("Example FSM");
 *
 *       // then you need commands
 *       Command cmd1 = Commands.runOnce(()->System.out.println("command 1 printed this one line.")).ignoringDisable(true);
 *       Command cmd2 = Commands.run(()->System.out.println("command 2 loops until interrupted."));
 *
 *       // next the commands create the states
 *       State state1 = stateMachine.addState("State 1", cmd1);
 *       State state2 = stateMachine.addState("state 2", cmd2);
 *
 *       // need an initial state at some point before running (first state made is the default if not otherwise set)
 *       stateMachine.setInitialState(state1);
 *
 *       // then you need conditions
 *       // These are external conditions for the "when". The condition for "whenComplete" is internal
 *       // and implied by the use of that method.
 *       BooleanSupplier condition1 = () -> (int) (Timer.getFPGATimestamp()*10. % 14.) == 0;
 *
 *       // the conditions determine the state changes
 *       state1.switchTo(state2).whenComplete();
 *       state2.switchTo(state1).when(condition1);
 *
 *       // Examples of a stop state and idle state that could have been used (but were not)
 *       State stop = stateMachine.addState("stop state", Commands.none().ignoringDisable(true)); // test message
 *       State idle = stateMachine.addState("idle state", Commands.idle().ignoringDisable(true)); // test message
 *   
 *       System.out.println(stateMachine);
 *   
 *       return stateMachine;
 * }
 * createStateMachine.schedule();
 *</code></pre>
 */
public class StateMachine extends Command {

  /////////////////////////////////////
  // "ONE-TIME" SETUP THE STATE MACHINE
  /////////////////////////////////////
  
  private String name;
  private boolean FSMfinished;
  private EventLoop events = new EventLoop();
  private List<State> states = new ArrayList<State>(); // All the states users instantiate (and STOP) only needed for printing the StateMachine - not in the logical flow
  private State initialState = null; // user calls setInitialState or else the first state made is the default initial state
  private State completedNormally = null; // used for internal transition trigger whenComplete
  private Command stateCommandAugmentedPrevious = null; // need to know if previous is still running so can be cancelled on state transition

  public StateMachine(String name) {
    this.name = name;
  }

  /**
   * Set the initial state else the first state made is the default initial state.
   *
   * @param initialState The new initial state. Cannot be null. The last initial state set before
   * scheduling the StateMachine is the effective initial state.
   */
  public void setInitialState(State initialState) {
    this.initialState = initialState;
  }

  /**
   * Associate a state and a command
   * 
   * @param name of the state
   * @param stateCommand command used to effect the state
   * @return the state
   */
  public State addState(String name, Command stateCommand) {
    var state = new State(name, stateCommand);
    if (initialState == null) { // first state made is the default initial state
      initialState = state;
    }
    return state;
  }

  /**
   * Print State and Transition information about the StateMachine
   * 
   * @return String of StateMachine information
   */
  public String toString() {
    StringBuilder sb = new StringBuilder();

    sb.append("All states for StateMachine " + name + "\n");
    
    for (State state : states) {
      boolean noExits = true; // initially haven't found any
      boolean noEntrances = true; // initially haven't found any

      sb.append("-------" + state.name + "-------" + (state == initialState ? " INITIAL STATE\n" : "\n"));

      // loop through all the transitions of this state
      for (Transition transition : state.transitions) {
        noExits = false; // at least one transition out of this state
        sb.append("transition " +
          transition + " to " + transition.nextState.name + " with trigger " + transition.triggeringEvent + "\n");
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
      sb.append(
        (noEntrances?"Caution - State has no entrances and will not be used.\n\n":
        noExits?"Notice - State has no exits and if entered will either stop or hang the StateMachine command.\n\n":"\n"));
    }
    return sb.toString();
  }

  /////////////////////////////////////////////////////
  // THE ITERATIVE CONTROL COMMAND OF THE STATE MACHINE
  /////////////////////////////////////////////////////
 
  /** Called once when the StateMachine command is scheduled. */
  public void initialize() {
    FSMfinished = false;
    events.clear(); // make sure clear in case there would be a race between the execute poll and the next command clear (maybe used if FSM can start/stop which it can't right now)
    new ScheduleCommand(initialState.stateCommandAugmented).schedule();
  }

  /** Called repeatedly while the StateMachine is running to check for triggering events. */
  @Override
  public void execute() {
    events.poll(); // check for events that can trigger transitions out of this state
  }

  /**
   * StateMachine is ending
   * @param interrupted whether the command was interrupted/canceled (not used)
   */
  @Override
  public void end(boolean interrupted) {
    // cancel the State command if it's still running
    if (stateCommandAugmentedPrevious != null) {
      stateCommandAugmentedPrevious.cancel();
    }
  }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the stateMachine command has been ordered to finish.
   */
  @Override
  public boolean isFinished() {
    return FSMfinished; // check if last state command ordered StateMachine to stop
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
     * All the rest of the states that run must be scheduled by an event.
     */
    @Override
    public void initialize() {
      events.clear(); // wipe the previous state's triggers
      if(stateCommandAugmentedPrevious != null) {
        stateCommandAugmentedPrevious.cancel(); // wipe the previous state in case it didn't finish itself
      }
       // make triggers for all of the current state's transitions
      if (state.transitions != null) {
        for (Transition transition : state.transitions) { //  add all the events for this state
          new Trigger (events, transition.triggeringEvent)
            .onTrue(transition.nextState.stateCommandAugmented);
        }
      }

      completedNormally = null; // reset internal trigger for this new state
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
      if (!interrupted) {
        completedNormally = state; // for internal trigger indicate state ended by itself without others help
      }
      stateCommandAugmentedPrevious = null; // indicate state ended (there is no effective previous to cancel)
      if (state.transitions.isEmpty()) {
        FSMfinished = true; // tell StateMachine to end since this was last command to run (no exit transitions)
      }
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
      public void when(BooleanSupplier condition) { // imply run until the external event then transition
        transitions.add(new Transition(m_targetState, condition)); // wrap condition and add to the list a transition to this state
      }

      /**
       * Marks the transition when the originating state completes without having reached any other
       * transitions first.
       */
      public void whenComplete() {
        BooleanSupplier condition = ()-> State.this == completedNormally; // internal trigger for command completion trigger
        transitions.add(new Transition(m_targetState, condition)); // wrap condition and add to the list a transition to this state
      }
    } // end class NeedsConditionTransitionBuilder
  } // end class State

  /**
   * class Transition is a Triggering external event to change to the next command (state)
   */
  private class Transition {
    State nextState; // also, essentially the "key" to finding a state
    BooleanSupplier triggeringEvent;

    private Transition(State toNextState, BooleanSupplier whenEvent) {
      this.nextState = toNextState;
      this.triggeringEvent = whenEvent; 
    }
  } // end class Transition
} // end class StateMachine
