package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.InternalButton;

/**
 * A command group that runs a list of commands in parallel after the primary command completes.
 * 
 * Each command runs relatively independently of the other commands.
 * 
 * Beware of conflicting requirements as there is no checking of that situation
 * before submittal to command scheduler which will enforce requirements as usual.
 * 
 * Compares to using "proxies" but uses "triggers"
 * 
 * 
 * Inspired by ChiefDelphi poster @bovlb
 */

/*

Usage:

    TriggeredDisjointParallelGroup.prepare(
        Commands.print("immediately printed"),
        waitSeconds(1).andThen(Commands.print("at 1 seconds")),
        waitSeconds(2).andThen(Commands.print("at 2 seconds")),
        waitSeconds(3).andThen(Commands.print("at 3 seconds")),
        waitSeconds(4).andThen(Commands.print("at 4 seconds")),    
        waitSeconds(5).andThen(Commands.print("at 5 seconds")),
        waitSeconds(6).andThen(Commands.print("at 6 seconds")),
        waitSeconds(7).andThen(Commands.print("at 7 seconds")),
        waitSeconds(8).andThen(Commands.print("at 8 seconds"))    
      ).schedule();

*/

/**
 * A command group that runs a list of commands in parallel after the primary command completes.
 *
 * <p>Because each component command is individually composed, some rules for command compositions
 * apply: commands that are passed to it cannot be added to any other composition or scheduled
 * individually unless first released by
 * CommandScheduler.getInstance().removeComposedCommand(test1).
 *
 * <p>The difference with regular group compositions is this sequential group does not require at
 * all time all of the subsystems its components require.
 */
public final class TriggeredDisjointParallelGroup extends WrapperCommand {

  private static InternalButton[] m_trigger = null;

  private TriggeredDisjointParallelGroup(Command command) {
    super(command);
  }

  @Override
  public void initialize() {
    for (int i = 0; i < m_trigger.length; i++) {
    m_trigger[i].setPressed(false);
    // reset in case this is reused (maybe by sloppy use
    // of not restarting robot code and just changing modes and
    // returning to a previous mode but it's supported)      
    }

    m_command.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
    for (int i = 0; i < m_trigger.length; i++) {
      m_trigger[i].setPressed(true);// indicate command ended and the rest of the commands are to be triggered     
    }
  }

  /**
   * When the first command in the list ends all the rest of the commands are triggered to run automatically.
   *
   * <p>The first command is added to an individual composition group (WrapperCommand) and thus is
   * restricted but the requirements of each component command are not required for the entire group
   * process since each wrapped command is run individually by being triggered from the previous
   * command.
   *
   * <p>Schedule the first command and upon completion all the rest trigger run automatically .
   *
   * @param commands - list of commands to run
   * @return the first command that should be scheduled to run and the remainder are automatically
   *     run upon completion of the first command.
   */
  public static Command prepare(Command... commands) {
    if (commands.length == 0) {
      return null; // nothing to do
    }

    if (commands.length == 1) {
      return commands[0]; // only one so no augmentation or triggers needed
    }

    // first command (0) will be triggered externally by the user
    // and all the rest of the commands are assigned triggers
    m_trigger = new InternalButton[commands.length-1];
    for (int i = 1; i < commands.length; i++) {
      m_trigger[i-1] = new InternalButton();
      m_trigger[i-1].onTrue(commands[i]);
    }

    return new TriggeredDisjointParallelGroup(commands[0]); // only the primary command is augmented to trigger the rest
  }
}
