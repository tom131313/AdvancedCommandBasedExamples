package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateMachine.State;
import frc.robot.subsystems.RobotSignals.LEDView;

/**
 * Demonstration of a Moore-Like FSM example based on the StateMachine class model that has the same
 * appearance as the StateMachine in WPILib Command-Based V3.
 * 
 * This FSM example sequentially displays eight red LEDs first to last then back last to first
 *   1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 7 -> 6 -> 5 -> 4 -> 3 -> 2 -> 1 -> 2 ...
 * 
 * To demonstrate the trigger for "whenComplete" a cycle counter state is added. It contributes
 * nothing to the light bar and is just to show the use of "whenComplete". The cycle count is
 * displayed inSmartDashboard.]
 * 
 * The triggers are a user specified clock period (1/10th second) distributed among 14 bins for 14
 * triggers needed for this example of the Knight Rider Kitt Scanner.
 * 
 * The scanner runs Disabled or Enabled and in the example usage in Robot it is started immediately.
 * 
 * This example is a bit of a cheat - that is there are a few things wrong with it not being a
 * perfect FSM. There are several complex states but they are all identical except for a sequence
 * number - light number. That allows severe compression of code.  Normally each state would have its
 * own Functional Command combining the Entry, Exit, and Steady-state Runnables for that state.
 * 
 * There 8 states of the lights and each of those states has 2 possible exit transitions for counting
 * up or counting down. An additional state to count cycles is defined to show an example of the
 * transition made if a state completes normally (internal event) and was not interrupted by an
 * external event.
 * 
 * This FSM does not demonstrate a STOP State except by cancelling the command. An example STOP is
 * commented out.
 */
public class MooreLikeFSM {

  private final LEDView m_robotSignals; // LED view where the output is displayed
  private double m_periodFactor; // changeable speed of the scanner
  private final Color m_color; // changeable color of the scanner
  private final double m_numberPeriods = 14.0; // number of periods or time bins to generate time-based triggers
  private int counter;

  /**
   * Eight states of the lights in the Knight Rider Kitt Scanner.
   * Caution - anti-pattern - the ordinal of the state is used as the hardware LED index (0 based).
   * That could be made more obvious by using a class variable for each state.
   * 
   * These states only roughly correspond to the States of the StateMachine as they are the light
   * patterns only and aren't used for the counter State.
   */ 
  private enum LightState
    {Light1, Light2, Light3, Light4, Light5, Light6, Light7, Light8};

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
    // start it in auto or periodic init with m_robotContainer.getM_mooreLikeFSM().get().schedule();
  }

  /**
   * Factory to create a new lightBar FSM using methods that appear similar to the Command-Based V3
   * implementation of StateMachine.
   * 
   * @return new lightBar FSM
   */
  public StateMachine createLightBar()
  {
    // With the StateMachine usage each transition belongs exclusively to the current state to exit.
    // The transition is the triggering condition and the next state to transition to.

    counter = 0; // something to display for the "whenComplete" example state

    var lightBar = new StateMachine("Kitt Light Bar Scanner");

    // first you need commands
    Command count = Commands.runOnce(()->SmartDashboard.putNumber("light bar cycles", ++counter)).ignoringDisable(true);
    Command activateLight1 = activateLight(LightState.Light1);
    Command activateLight2 = activateLight(LightState.Light2);
    Command activateLight3 = activateLight(LightState.Light3);
    Command activateLight4 = activateLight(LightState.Light4);
    Command activateLight5 = activateLight(LightState.Light5);
    Command activateLight6 = activateLight(LightState.Light6);
    Command activateLight7 = activateLight(LightState.Light7);
    Command activateLight8 = activateLight(LightState.Light8);


    // then the commands create the states
    State countCycles = lightBar.addState("count cycles", count);
    State light1 = lightBar.addState("light1", activateLight1);
    State light2 = lightBar.addState("light2", activateLight2);
    State light3 = lightBar.addState("light3", activateLight3);
    State light4 = lightBar.addState("light4", activateLight4);
    State light5 = lightBar.addState("light5", activateLight5);
    State light6 = lightBar.addState("light6", activateLight6);
    State light7 = lightBar.addState("light7", activateLight7);
    State light8 = lightBar.addState("light8", activateLight8);

    // need an initial state at some point before running
    lightBar.setInitialState(countCycles);

    // then you need conditions
    // These are external conditions for the "when". The condition for "whenComplete" is internal
    // and implied by the use of that method.
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
    countCycles.switchTo(light1).whenComplete();
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
    light2.switchTo(countCycles).when(period13);
    //there is no end State defined so keep scanning until the FSM is cancelled.
  
    lightBar.printStateMachine();

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
          })
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
