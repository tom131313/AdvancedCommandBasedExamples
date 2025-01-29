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
 * Somewhat similar to using "proxies" but uses "triggers"
 * 
 * Inspired by ChiefDelphi poster @bovlb

Usage:

    new TriggeredDisjointParallelGroup(
        Commands.print("immediately printed"),
        waitSeconds(6).andThen(Commands.print("at 6 of 6 seconds")),
        waitSeconds(5).andThen(Commands.print("at 5 of 6 seconds")),
        waitSeconds(4).andThen(Commands.print("at 4 of 6 seconds")),
        waitSeconds(1).andThen(Commands.print("at 1 of 6 seconds")),    
        waitSeconds(2).andThen(Commands.print("at 2 of 6 seconds")),
        waitSeconds(3).andThen(Commands.print("at 3 of 6 seconds"))
      ).schedule();
*/

public final class TriggeredDisjointParallelGroup extends WrapperCommand {

  private final InternalButton m_trigger ;
  private final boolean continueOnInterrupt;

  /**
   * When the first command in the list ends all the rest of the commands are triggered to run automatically.
   *
   * <p>The first command is added to an individual composition group (WrapperCommand) and thus is
   * restricted but the requirements of each component command are not required for the entire group
   * process since each wrapped command is run individually by being triggered from the previous
   * command.
   *
   * <p>Schedule the first command and upon completion all the rest trigger to run automatically .
   * @param continueOnInterrupt - true is subsequent commands are triggered even if primary command is interrupted;
   *                              false is commands do not run if primary command is interrupted
   * @param commands - list of commands to run
   * @return the first command that should be scheduled to run and the remainder are automatically
   *     run upon completion of the first command.
   */
    public TriggeredDisjointParallelGroup(boolean continueOnInterrupt, Command... commands) {
    super(commands[0]);
    this.continueOnInterrupt = continueOnInterrupt;
    m_trigger = new InternalButton();
    // first command [0] will be triggered externally by the user
    // and all the rest of the commands are assigned triggers
    // to run upon completion of the first command
    for (int i = 1; i < commands.length; i++) {
      m_trigger.onTrue(commands[i]);
    }
  }

  @Override
  public void initialize() {
    
    m_trigger.setPressed(false);
    // reset in case this is reused (maybe by sloppy use
    // of not restarting robot code and just changing modes and
    // returning to a previous mode but it's supported)      

    m_command.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
    if (continueOnInterrupt || !interrupted) {
      m_trigger.setPressed(true); // indicate command ended and the rest of the commands are to be triggered     
    }
  }
}
