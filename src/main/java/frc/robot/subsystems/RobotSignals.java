package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manage the addressable LEDs as signaling subsystems.
 *
 * <p>This is the creator and container of the LEDView subsystems.
 *
 * <p>Buffer is not cleared.
 * 
 * <p>An alternative implementation that deserves consideration for most uses is:
 *   make the default command "black, off"
 *   delete the "setSignalOnce" methods since the default off would trigger immediately
 *   schedule "setSignal" which persists until interrupted
 * 
 * <p>Another possibility is only use the default command and supply a differing pattern as desired
 * for differing signals. That loses the advantages of Command management interrupts, however, since
 * there is no similar management of LED Patterns.
 */

/*
 * All Command factories are "public."
 *
 * All other methods are "private" to prevent other classes from forgetting to add requirements of
 * these resources if creating commands from these methods.
 */

public class RobotSignals {
  /**
   * Represents a supplier of LEDPattern for creating dynamic LEDPatterns.
   *
   * <p>Can't overload methods using generic interfaces parameters like Supplier so make our own
   * interface to use in overloads
   */
  @FunctionalInterface
  public interface LEDPatternSupplier {
    /**
     * Gets a result.
     *
     * @return a result
     */
    LEDPattern get();
  }

  private final AddressableLED m_strip;
  private final AddressableLEDBuffer m_bufferLED;
  private static int m_length = 0; // length of the buffer - last LED used + 1 for the number 0 LED

  // location in the LED string is defined and reserved for all examples even if not selected to run
  public final LEDView m_top;
  public final LEDView m_main;
  public final LEDView m_enableDisable;
  public final LEDView m_historyDemo;
  public final LEDView m_achieveHueGoal;
  public final LEDView m_knightRider;
  public final LEDView m_imposter;

  /**
   * Layout by LED number of the single physical buffer into multiple logical views or resources/subsystems.
   * 
   * Location of view in buffer - zero-based buffer numbering.
   * 
   * Order doesn't matter.

   * A view will be reversed if the starting index is after the
   * ending index; writing front-to-back in the view will write
   * in the back-to-front direction on the underlying buffer.
   *
   * You take your chances with overlapping views.
   */
  private static enum LEDViewPlacement {
    TOP           (0, 7),
    MAIN          (8, 15),
    ENABLEDISABLE (16, 23),
    HISTORYDEMO   (24, 31),
    ACHIEVEHUEGOAL(32, 39),
    KNIGHTRIDER   (40, 47),
    IMPOSTER      (48, 55);
  
    public int first;
    public int last;

    /**
     * Location of view in buffer [zero-based numbering]
     * 
     * @param first LED number inclusive
     * @param last LED number inclusive
     */
    private LEDViewPlacement(int first, int last)
    {
      this.first = first;
      this.last = last;
    }
  }

  public RobotSignals() {

    // find number of LEDs used
    for(LEDViewPlacement index : LEDViewPlacement.values())
    {
      m_length = Math.max(m_length, index.last + 1); // position is zero-based; + 1 for length
    }

    // start updating the physical LED strip
    final int addressableLedPwmPort = 1;
    m_strip = new AddressableLED(addressableLedPwmPort);
    m_strip.setLength(m_length);
    m_strip.start();
    m_bufferLED = new AddressableLEDBuffer(m_length); // buffer for all of the LEDs

    // create the resources (subsystems) as views of the LED buffer
    m_top            = new LEDView(LEDViewPlacement.TOP);
    m_main           = new LEDView(LEDViewPlacement.MAIN);
    m_enableDisable  = new LEDView(LEDViewPlacement.ENABLEDISABLE);
    m_historyDemo    = new LEDView(LEDViewPlacement.HISTORYDEMO);
    m_achieveHueGoal = new LEDView(LEDViewPlacement.ACHIEVEHUEGOAL);
    m_knightRider    = new LEDView(LEDViewPlacement.KNIGHTRIDER);
    m_imposter       = new LEDView(LEDViewPlacement.IMPOSTER);
  }

  /**
   * Run before commands and triggers
   */
  public void runBeforeCommands() {}

  /**
   * Run after commands and triggers
   */
  public void runAfterCommands() {
    m_strip.setData(m_bufferLED); // run periodically to send the buffer to the LEDs
  }

  /** LED view resource (subsystem) */
  public class LEDView extends SubsystemBase {

    private final AddressableLEDBufferView m_view;

    private LEDView(LEDViewPlacement placement) {
      m_view = m_bufferLED.createView(placement.first, placement.last);
    }

    /*
     * Public Commands
     */

    /**
     * Example of how to allow one (or none) default command to be set.
     *
     * @param def default command
     */
    @Override
    public void setDefaultCommand(Command def) {
      if (getDefaultCommand() != null) {
        throw new IllegalArgumentException("Default Command already set");
      }

      if (def != null) {
        super.setDefaultCommand(def);
      }
    }

    // Some uses of setSignal assume the command keeps running so provide that function.
    // Some uses of setSignal refresh the signal in their own looping so provide a single use, too.

    /**
     * Put an LED Pattern into the view - keep running.
     *
     * @param pattern
     * @return Command to apply pattern to LEDs
     */
    public Command setSignal(LEDPattern pattern) {
      return run(() -> pattern.applyTo(m_view)).ignoringDisable(true).withName("LedSet");
    }

    /**
     * Put a dynamic (supplied) LED Pattern into the view - keep running.
     *
     * @param pattern
     * @return Command to apply pattern to LEDs
     */
    public Command setSignal(LEDPatternSupplier pattern) {
      return run(() -> pattern.get().applyTo(m_view)).ignoringDisable(true).withName("LedSetS");
    }

    /**
     * Put an LED Pattern into the view - once.
     * 
     * Not recommended and not for animated patterns such as "blink"
     *
     * @param pattern
     * @return Command to apply pattern to LEDs
     */
    public Command setSignalOnce(LEDPattern pattern) {
      return runOnce(() -> pattern.applyTo(m_view)).ignoringDisable(true).withName("LedSetO");
    }

    /**
     * Put a dynamic (supplied) LED Pattern into the view - once.
     * 
     * Not recommended and not for animated patterns such as "blink"
     * 
     * @param pattern
     * @return Command to apply pattern to LEDs
     */
    public Command setSignalOnce(LEDPatternSupplier pattern) {
      return runOnce(() -> pattern.get().applyTo(m_view)).ignoringDisable(true).withName("LedSetSO");
    }
  } // End LEDView
}
