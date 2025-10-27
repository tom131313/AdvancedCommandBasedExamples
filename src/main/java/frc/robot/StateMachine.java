package frc.robot;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// @SuppressWarnings("unused")
/**
 * Example of a Finite State Machine (FSM) using simple methods to build the FSM. The syntax is
 * essentially similar to typical FSM documentation.
 * 
 * <p>Based on the user-facing appearance of Command-Based V3 (as of 10/2025).
 * 
 * <p>This is similar to typical coding of Triggers with conditions and onTrue commands. A benefit
 * of this FSM implementation is the state-changing triggers exist only for the duration of the state
 * instead of being a perpetual part of the huge mass of triggers for the robot code.
 * 
 * <p>Another feature is an automatically created internal trigger for when a state command completes
 * normally instead of being interrupted. Use "whenComplete()" to use that feature. Use "when()" for a
 * typical external trigger condition.
 * 
 * <p>Multiple transitions with the same effective conditions are effectively undefined actions. Both
 * transitions may be triggered in quick succession. There is validation to prevent using a condition
 * object more than once but there is no way to prevent using the effectively identical condition in
 * more than one object. WARNING - Do not specify two different transitions with effectively the same
 * condition. There is no way to prevent programmatically a different condition object that triggers
 * on the same logic. The user must get this right. A warning message is attempted to be provided if
 * there are multiple transitions triggered simultaneously by different but functionally identical
 * conditions. (Providing a replacement event polling mechanism could prevent this occurrence but this
 * code is trying to use standard WPILib Command-Based V2 functions.)
 * 
 * <p>Duplicate conditions in the V3 2027 WPILib implementation may behave better but with the first
 * one being the one that is used (V3 is structured very differently - better than V2).
 * 
 * <p>Any state without a transition is an exit state if entered and completes; or hangs the
 * stateMachine if it doesn't complete. A purposeful exit can be coded with
 * "somestate.exitStateMachine().when(somecondition);" or "somestate.exitStateMachine().whenComplete();"
 * 
 * <p>The StateMachine does not have an idle state. Any state entered and does nothing until interrupted
 * would be idle for its duration. Example idle state shown below could be used to keep the StateMachine
 * running so it does not end and would not need to be recreated, say from a factory, for a restart.
 * 
 * <p>Command-Based classes are used to wrap the users commands and triggers in order to define the
 * FSM "cyclic" or "branching" behavior. The general flow is: the user builds the StateMachine with
 * states and transition conditions that trigger the next state. StateMachine command initialize
 * schedules the initial (start) state. The StateMachineCommand execute polls the event loop looking
 * to start the next the command for the next state. The next state (of the initial state) initialize
 * cleans up the previous state and builds its event triggers. A special variable indicates if a state
 * has completed normally. A special variable indicates if the StateMachine is the exit (end).
 * 
 * <p>This code has incomplete validation to prevent all really bad parameters. There is some validation
 * of inappropriate use of nulls, duplicate usage of condition objects, and duplicate conditions in
 * different objects for a single state. The anticipated V3 implementation has much better validation
 * against things you shouldn't do.
 * 
 * <p>Some of the transition builder code was copied from an early version of the V3 StateMachine and
 * may have some statements that aren't pertinent to this V2 implementation. Some of it has been
 * changed already for V3 but wasn't changed herein for V2.
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
 *       Command cmd1 = Commands.runOnce(()-> System.out.println("command 1 printed this one line.")).ignoringDisable(true);
 *       Command cmd2 = Commands.run(()-> System.out.println("command 2 loops until interrupted.")); // note this won't run disabled
 *
 *       // next the commands are used to create the states
 *       State state1 = stateMachine.addState("State 1", cmd1);
 *       State state2 = stateMachine.addState("state 2", cmd2);
 *
 *       // require an initial state at some point before scheduling
 *       stateMachine.setInitialState(state1);
 *
 *       // then you need conditions for state-changing triggers
 *       // These are external conditions for the "when".
 *       // The condition for "whenComplete" is internal and implied by the use of that method.
 *       BooleanSupplier condition1 = () -> (int) (Timer.getFPGATimestamp()*10. % 14.) == 0;
 *
 *       // the conditions determine when the states change
 *       state1.switchTo(state2).whenComplete();
 *       state2.switchTo(state1).when(condition1);
 *       state2.exitStateMachine.when(condition2); // exit (end) the FSM - optional as needed for the FSM logic
 *
 *       // Example of a not recommended stop state that could have been used (but was not)
 *       State stop = stateMachine.addState("stop state", Commands.none().ignoringDisable(true)); // better to use "exitStateMachine"; test message
 * 
 *       // Example of an idle state that could have been used (but was not)
 *       State idle = stateMachine.addState("idle state", Commands.idle().ignoringDisable(true)); // test message
 *   
 *       System.out.println(stateMachine); // optional use of the toString()
 *   
 *       return stateMachine; // the FSM command produced by this factory
 * }
 * createStateMachine().schedule(); // scheduling the FSM starts it running immediately
 *</code></pre>
 */
public class StateMachine extends Command {

  /////////////////////////////////////
  // "ONE-TIME" SETUP THE STATE MACHINE
  /////////////////////////////////////
  
  private String name = "not instantiated"; // name of the FSM
  private boolean exitStateMachine = false; // flag signals if FSM is to exit (end)
  private EventLoop events = new EventLoop(); // polled for triggers
  private List<State> states = new ArrayList<State>(); // the instantiated states; only used for printing the StateMachine - not in the logical flow
  private State initialState = null; // user must call setInitialState before scheduling the FSM
  private State completedNormally = null; // flag for whenComplete() trigger
  private Command stateCommandAugmentedPrevious = null; // need to know if previous is still running so can be cancelled on state transition
  private int countSimultaneousTransitions = 0; // check for multiple simultaneous transition triggers

  public StateMachine(String name) {
    requireNonNullParam(name, "name", "StateMachine");
    this.name = name;
  }

  /**
   * Sets the initial (start) state for the state machine.
   * This state runs immediately after scheduling the state machine command.
   *
   * @param initialState The new initial state. Cannot be null.
   */
  public void setInitialState(State initialState) {
    requireNonNullParam(initialState, "initialState", "StateMachine.setInitialState");
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
    return new State(name, stateCommand);
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

      sb.append("-------" + state.name + "-------\n");
      sb.append(state == initialState ? "INITIAL STATE\n" : "");
      // loop through all the transitions of this state
      for (Transition transition : state.transitions) {
        noExits = false; // at least one transition out of this state
        sb.append("transition " +
          transition + " to " + (transition.nextState != null ? transition.nextState.name : "exit StateMachine") + " onTrue trigger " + transition.triggeringEvent + "\n");
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
        (noEntrances && state != initialState ? "Caution - State has no entrances and will not be used.\n\n" :
        noExits ? "Notice - State has no exits and if entered will either stop or hang the StateMachine command.\n\n" : "\n"));
    }
    return sb.toString();
  }

  /////////////////////////////////////////////////////
  // THE ITERATIVE CONTROL COMMAND OF THE STATE MACHINE
  /////////////////////////////////////////////////////
 
  /** Called once when the StateMachine command is scheduled. */
  @Override
  public void initialize() {
    exitStateMachine = false;
    initialState.stateCommandAugmented.schedule();
  }

  /** Called repeatedly while the StateMachine is running to check for triggering events. */
  @Override
  public void execute() {
    if (countSimultaneousTransitions > 1) {
      DriverStation.reportWarning("Multiple states triggered simultaneously", false);
    }
    countSimultaneousTransitions = 0;
    events.poll(); // check for events that trigger transitions
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
    return exitStateMachine; // check if last state command ordered StateMachine to stop
  }

  /////////////////////////////////////
  // RUN THE STATES AS WRAPPED COMMANDS
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
     * This is the beginning of a running state because somebody scheduled it.
     * [The initial (start) state was scheduled when the StateMachine started.
     * All the rest of the states that run must be scheduled by an event.]
     */
    @Override
    public void initialize() {
      events.clear(); // wipe the previous state's triggers
      if(stateCommandAugmentedPrevious != null) {
        stateCommandAugmentedPrevious.cancel(); // wipe the previous state in case it didn't finish itself
      }
      // make triggers for all of this state's transitions
      // if no transitions, that will be handled later as an exit but first need to run this state
      if ( ! state.transitions.isEmpty()) {
        for (Transition transition : state.transitions) { // add all the events for this state
          var trigger = new Trigger (events, transition.triggeringEvent); // for .when(condition) and .whenComplete()
          trigger.onTrue(Commands.runOnce(()-> ++countSimultaneousTransitions).ignoringDisable(true)); // for check erroneous multiple identical conditions
          if (transition.nextState == null) { // condition for .exitStateMachine()
            trigger.onTrue(Commands.runOnce(()-> exitStateMachine = true).ignoringDisable(true)); // flag to exit (end) FSM
          }
          else { // condition to trigger next state
            trigger.onTrue(transition.nextState.stateCommandAugmented); // start next state
          }
        }
      }

      completedNormally = null; // reset flag for this new state as it has not yet completed normally 'cuz it's just starting
      stateCommandAugmentedPrevious = this; // for next state change this will be the previous state

      m_command.initialize(); // Wrapper is done with its fussing so tell original command to initialize
    }

  /**
    * The action to take when the command ends. Called when either the command finishes normally, or
    * when it is interrupted/canceled.
    * @param interrupted whether the command was interrupted/canceled
    */
    @Override
    public void end(boolean interrupted) {
      m_command.end(interrupted); // tell original command to end and if interrupted or not

      // setup for the next state or exit
      stateCommandAugmentedPrevious = null; // indicate state already ended so there is not a previous state to cancel

      if (state.transitions.isEmpty()) { // no transitions [no .when() nor .whenComplete()]
        exitStateMachine = true; // no matter how this state ended tell StateMachine to exit since nowhere to go from here
      }
      else {
        if (!interrupted) {
          completedNormally = state; // indicate state ended by itself without others help
          // see if this state has transition .exitStateMachine().whenComplete()
          for (Transition transition : state.transitions) { // check all transitions
            if (transition.triggeringEvent == state.whenCompleteCondition) { // for .whenComplete()
              if (transition.nextState == null) { // for .exitStateMachine()
                exitStateMachine = true;
              }
              break; // don't look for any more since cannot be more than one whenComplete trigger
            }
          }
        }
      }
    }
  } // end class WrapState

  /**
   * class State as a command with exit transitions (event + next state command)
   */
  public class State extends Command
  {
    private final String name;
    private Command stateCommandAugmented; // the Wrapped (instrumented) state command that will actually be run
    private List<Transition> transitions = new ArrayList<Transition>(); // the transitions for this State
    private BooleanSupplier whenCompleteCondition = ()-> State.this == completedNormally; // trigger condition for whenComplete

    /**
     * creating a new State from a command
     * @param name 
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
    public TransitionNeedsConditionStage switchTo(State to) {
      requireNonNullParam(to, "to", "State.switchTo");
      return new TransitionNeedsTargetStage(this).to(to);
    }

    /**
     * Starts building a transition that will exit the state machine when triggered, rather than
     * moving to a different state.
     *
     * @return A builder for the transition.
     */
    public TransitionNeedsConditionStage exitStateMachine() {
      return new TransitionNeedsConditionStage(this, null);
    }

    /**
     * A builder for a transition from one state to another. Use {@link #to(State)} to specify the
     * target state to transition to.
     */
    public final class TransitionNeedsTargetStage {
      private final State m_from;

      private TransitionNeedsTargetStage(State from) {
        m_from = from;
      }

      /**
       * Specifies the target state to transition to.
       *
       * @param to The state to transition to. Cannot be null.
       * @return A builder to specify the transition condition.
       */
      private TransitionNeedsConditionStage to(State to) {
        return new TransitionNeedsConditionStage(m_from, to);
      }
    } // end class NeedsTargetTransitionBuilder

    /**
     * A builder to set conditions for a transition from one state to another. Use {@link
     * #when(BooleanSupplier)} to make the transition occur when some external condition becomes true,
     * or use {@link #whenComplete()} to make the transition occur when the originating state
     * completes without having reached any other transitions first.
     */
    public final class TransitionNeedsConditionStage {
      // private final State m_originatingState;
      private final State m_targetState;

      private TransitionNeedsConditionStage(State from, State to) {
        // m_originatingState = from;
        m_targetState = to;
      }

      /**
       * Adds a transition that will be triggered when the specified condition is true.
       *
       * @param condition The condition that will trigger the transition.
       */
      public void when(BooleanSupplier condition) {
        checkDuplicateCondition(condition);
        transitions.add(new Transition(m_targetState, condition)); // wrap condition and add to the list a transition to this state
      }

      /**
       * Marks the transition when the originating state completes without having reached any other
       * transitions first.
       */
      public void whenComplete() {
        checkDuplicateCondition(whenCompleteCondition);
        transitions.add(new Transition(m_targetState, whenCompleteCondition)); // wrap condition and add to the list a transition to this state
      }

      /**
       * Prevent a condition object from being used in more than one transition per state.
       * 
       * <p>This check cannot prevent effectively identical conditions in different objects from
       * being used. The user must assure that two or more conditions in a state will not trigger
       * at the same time. The result of two identical conditions is essentially undefined. There
       * is an attempt to detect but not prevent this condition at runtime.
       * 
       * @throws IllegalArgumentException if a condition object is reused in a single state.
       */
      private void checkDuplicateCondition(BooleanSupplier condition) {
        for (Transition transition : transitions) {
          if (transition.triggeringEvent == condition) {
            throw new IllegalArgumentException("Condition object can be used only once per state.");
          }
        }
      }
    } // end class NeedsConditionTransitionBuilder
  } // end class State

  /**
   * Transition is a triggering event causes a change from the current state to the next state
   */
  private class Transition {
    State nextState;
    BooleanSupplier triggeringEvent;

    /**
     * Define the FSM transition as current state + triggering event -> next state
     * 
     * @param toNextState next state or null means exit state machine (no next state)
     * @param whenEvent the when and whenComplete conditions
     */
    private Transition(State toNextState, BooleanSupplier whenEvent) {
      this.nextState = toNextState;
      this.triggeringEvent = whenEvent; 
    }
  } // end class Transition

  /**
   * Another StateMachine test
   * <p>uses digital input 0 for some state changes
   * <p>usage:
   * <pre><code>
   * StateMachine.StateMachineTest.testFSM().schedule();
   * </code></pre>
   */
  public class StateMachineTest extends Command {
      private int count;
      private String name;
  
      public StateMachineTest(String name) {
          this.name = name;
      }
  
      @Override
      public void initialize() {
          count = 0;
          System.out.println(name + " " + count + " initialize");
      }
  
      @Override
      public void execute() {
          ++count;
          System.out.println(name + " " + count);
      }
      
      @Override
      public void end(boolean interrupt) {
          System.out.println(name + " " + count + " end");
      }
  
      @Override
      public boolean isFinished() {
          if (name.startsWith("unlimited")) {
              return false;
          }
          else {
              return count >= 10;            
          }
      }
  
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
  
      static DigitalInput di = new DigitalInput(0);
      
      public static Command testFSM() {
          StateMachine tester = new StateMachine("test machine");
  
          State state1 = tester.addState("state1", tester.new StateMachineTest("command1"));
          State state2 = tester.addState("state2", tester.new StateMachineTest("command2"));
          State state3 = tester.addState("state3", tester.new StateMachineTest("command3"));
          State state4 = tester.addState("state4", tester.new StateMachineTest("command4"));
  
          State state5 = tester.addState("state5", tester.new StateMachineTest("unlimited5"));
          State state6 = tester.addState("state6", tester.new StateMachineTest("unlimited6"));
  
          State state7 = tester.addState("state7", tester.new StateMachineTest("command7"));

          state1.switchTo(state2).whenComplete();
          // state1.exitStateMachine().whenComplete(); // uncomment to test throw error on duplicate condition object
          state2.switchTo(state3).whenComplete();
          state3.switchTo(state4).whenComplete();
          state4.switchTo(state5).whenComplete();
  
          BooleanSupplier condition1 = ()-> di.get();
          @SuppressWarnings("unused")
          BooleanSupplier condition2 = ()-> di.get();
          state5.switchTo(state6).when(condition1);
          // state5.switchTo(state7).when(condition2); // uncomment to test run time multiple triggers warning message on change of di 0 from low to high; results are bad and vary; don't do this
          // state5.switchTo(state7).when(condition1); // uncomment to test throw error on duplicate condition object
          state6.switchTo(state7).when(condition1);
  
          state7.exitStateMachine().whenComplete();

          tester.setInitialState(state1);

          System.out.println(tester);
          return tester.ignoringDisable(true);
      }
  } // end class StateMachineTest
} // end class StateMachine
