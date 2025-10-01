package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.RobotSignals.LEDView;
import frc.robot.subsystems.StateMachine.State;

/**
 * Demonstration of a Moore-Like FSM example that is similar to composing sequential and parallel
 * command groups. Triggers are used to control state selection instead of other commands and
 * decorators.
 * 
 * This FSM example sequentially displays eight red LEDs first to last then back last to first
 *   1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 7 -> 6 -> 5 -> 4 -> 3 -> 2 -> 1 -> 2 ...
 * 
 * The triggers are 1/10 second clock divided into 14 bins for 14 triggers needed for this example
 * of the Knight Rider Kitt Scanner.
 * 
 * The scanner runs Disabled and Enabled so the FSM is started immediately.
 * 
 * This example is a bit of a cheat - that is there are a few things wrong with it not being a
 * perfect FSM. There are several complex states but they are all identical except for a sequence
 * number. That allows severe compression of code.  Normally each state would have its own Functional
 * Command combining the Entry, Exit, and Steady-state Runnables for that state. The designation of
 * what is a state is subject to interpretation. There are 14 timed states and OFF. Or are there
 * 8 states of the lights (plus OFF) and each of those states has 2 possible exit transitions for
 * counting up or counting down. This example muddies the waters in this regard. This example
 * unnecessarily has a memory of what state it is in. The triggers by timed periods know their state
 * but if the scheme of having 8 states is used then each state needs to know itself so the exit
 * transition can correctly select for counting up or counting down. Real FSM usage requires better
 * design and not combine two schemes just for illustrative purposes.
 * 
 * 
 * This Moore-Like FSM is initially inactive and defines an Initial State when the FSM is activated.
 * 
 * Each state is composed of a State Entry Action, "Steady-State" Action and State Exit Action.
 * 
 * Each state waits for a transition requiring the state to exit.
 * 
 * A Transition from State to State is defined as the Current State + Trigger Condition yields Next
 * State.
 * 
 * This FSM does not demonstrate an STOP State except by cancelling the command.
 */
public class MooreLikeFSM {

  private final LEDView m_robotSignals; // LED view where the output is displayed
  private double m_periodFactor; // changeable speed of the scanner
  private final Color m_color; // changeable color of the scanner
  private final double m_numberPeriods = 14.0; // number of periods or time bins to generate time-based triggers
  public StateMachine lightBar;

  /**
   * Eight state FSM for the eight lights in the Knight Rider Kitt Scanner.
   * Caution - anti-pattern - the ordinal of the state is used as the hardware LED index (0 based).
   * That could be made more obvious by using a class variable for each state.
   */ 
  private enum LightState
    {Light1, Light2, Light3, Light4, Light5, Light6, Light7, Light8, Inactive};

  /**
   * A Moore-Like FSM to display lights similar to the Knight Rider Kitt Scanner
   * 
   * @param the LED View for the Scanner
   * @param periodFactor Specify the speed of the Scanner (suggest about 10.0)
   * @param color Specify the color of the Scanner (suggest Color.kRed)
   */

  public MooreLikeFSM(LEDView robotSignals, double periodFactor, Color color) {
    m_robotSignals = robotSignals;
    m_periodFactor = periodFactor;
    m_color = color;
    lightBar = createLightBar();
    lightBar.ignoringDisable(true).schedule(); // This FSM runs also disabled so start it immediately.
    //  If the FSM doesn't run disabled, then start it in auto or periodic init with
    // m_robotContainer.getM_mooreLikeFSM().get().lightBar.schedule();

    // example doesn't work to restart StateMachine. It does cancel but then fails to schedule
    // could try to restructure have createLightBar public and return a Command
    try {
      Thread.sleep(10000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    //FIXME scanner doesn't appear to stop but a second createLightBar does print out the state machine
    lightBar.cancel();
    lightBar = createLightBar();
    lightBar.ignoringDisable(true).schedule();
  }

  /**
   * Create an FSM using methods that appear similar to the Command-Based V3 implementation of
   * StateMachine.
   * 
   * <p>Use of ignoreDisabled requires using the requirement from the StateMachine (see ActivateLight
   * command example)
   */
  public StateMachine createLightBar()
  {
    // Each transition is the current state to exit AND a timed event period that together
    // trigger a command to attain the next state.
    // For this contrived example that's rather silly. The time period completely defines the state
    // and knowing the current state is completely unnecessary and extraneous but included to show
    // it can be done if that's how an FSM is defined.
    lightBar = new StateMachine("Kitt Light Bar Scanner");
    // first you need commands
    Command activateLight1 = activateLight(LightState.Light1);
    Command activateLight2 = activateLight(LightState.Light2);
    Command activateLight3 = activateLight(LightState.Light3);
    Command activateLight4 = activateLight(LightState.Light4);
    Command activateLight5 = activateLight(LightState.Light5);
    Command activateLight6 = activateLight(LightState.Light6);
    Command activateLight7 = activateLight(LightState.Light7);
    Command activateLight8 = activateLight(LightState.Light8);
    // then the commands create the states
    State light1 = lightBar.addState("light1", activateLight1);
    State light2 = lightBar.addState("light2", activateLight2);
    State light3 = lightBar.addState("light3", activateLight3);
    State light4 = lightBar.addState("light4", activateLight4);
    State light5 = lightBar.addState("light5", activateLight5);
    State light6 = lightBar.addState("light6", activateLight6);
    State light7 = lightBar.addState("light7", activateLight7);
    State light8 = lightBar.addState("light8", activateLight8);
    // need an initial state at some point before running
    lightBar.setInitialState(light1);
    // then you need conditions
    BooleanSupplier period0 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 0;
    BooleanSupplier period1 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 1;
    BooleanSupplier period2 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 2;
    BooleanSupplier period3 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 3;
    BooleanSupplier period4 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 4;
    BooleanSupplier period5 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 5;
    BooleanSupplier period6 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 6;
    BooleanSupplier period7 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 7;
    BooleanSupplier period8 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 8;
    BooleanSupplier period9 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 9;
    BooleanSupplier period10 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 10;
    BooleanSupplier period11 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 11;
    BooleanSupplier period12 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 12;
    BooleanSupplier period13 = () -> (int) (Timer.getFPGATimestamp()*m_periodFactor % m_numberPeriods) == 13;
    // the conditions determine the state changes
    light1.switchTo(light2).when(period0);
    light2.switchTo(light3).when(period1);
    light3.switchTo(light4).when(period2);
    light4.switchTo(light5).when(period3);
    light5.switchTo(light6).when(period4);
    light6.switchTo(light7).when(period5);
    light7.switchTo(light8).when(period6);
    light8.switchTo(light7).when(period7);
    light7.switchTo(light6).when(period8);
    light6.switchTo(light5).when(period9);
    light5.switchTo(light4).when(period10);
    light4.switchTo(light3).when(period11);
    light3.switchTo(light2).when(period12);
    light2.switchTo(light1).when(period13);
    // light8.switchTo(lightBar.stop).when(period7); // test stop state
  
    lightBar.printStateMachine();
    // There is no end State defined so keep scanning until the FSM is cancelled.
    return lightBar;
  }

  /**
   * Factory for commands that turn on the correct LED every iteration until interrupted by a new time period.
   * 
   * <p>Commands can't be put into the State enum because
   * enums are static and these commands in general are non-static especially with the
   * "this" subsystem requirement.
   * 
   * <p>Generally Command factories can be "public" but this is dedicated to this FSM and there is
   * no intention of allowing outside use of it as that can disrupt the proper function of the FSM.
   * 
   * @param state the state to enter
   * @return the command to run that defines the state - turns on the correct LED
   */
  private final Command activateLight(LightState state) {
    return 
      // steady-state action
        Commands.run(() ->
          {
            LEDPattern currentStateSignal = oneLEDSmeared(state.ordinal(), m_color, Color.kBlack);
            m_robotSignals.setSignal(currentStateSignal).schedule();
            SmartDashboard.putString("FSM steady-state action "+this, state.name());
          },
          lightBar.requirement)
        .ignoringDisable(true)
        .withName("Moore-Like " + m_color + " " + state); // "this" is more precise discriminator
                                                  // but "m_color" is prettier and likely as good
  }
 
  /**
   * Turn on one bright LED in the string view.
   * Turn on its neighbors dimly. It appears smeared.
   * 
   * A simple cheat of the real Knight Rider Kitt Scanner which has a slowly
   * diminishing comet tail.  https://www.youtube.com/watch?v=usui7ECHPNQ
   * 
   * @param index which LED to turn on
   * @param colorForeground color of the on LED
   * @param colorBackground color of the off LEDs
   * @return Pattern to apply to the LED view
   */
  private static final LEDPattern oneLEDSmeared(int light, Color colorForeground, Color colorBackground) {
    int index = light;
    final int slightlyDim = 180;
    final int dim = 120;

    return (reader, writer) -> {
      int bufLen = reader.getLength();

      for (int led = 0; led < bufLen; led++) {
        if (led == index) {
          writer.setLED(led, colorForeground);              
        } else if ((led == index-2 && index-2 >= 0) || (led == index+2 && index+2 < bufLen)) {
          writer.setRGB(led,
           (int) (colorForeground.red * dim),
           (int) (colorForeground.green * dim),
           (int) (colorForeground.blue * dim));
        } else if ((led == index-1 && index-1 >= 0) || (led == index+1 && index+1 < bufLen)) {
          writer.setRGB(led,
           (int) (colorForeground.red * slightlyDim),
           (int) (colorForeground.green * slightlyDim),
           (int) (colorForeground.blue * slightlyDim));
        } else {
          writer.setLED(led, colorBackground);              
        }
      }
    };
  }
}
/*

 */