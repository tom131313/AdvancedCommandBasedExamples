package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.Optional;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Create subsystems, define command logging, manage the details of what is
 * periodically processed before and after the command scheduler loop - everything
 * until it got too big and some logical splits to other classes had to be made.
 */

import frc.robot.subsystems.AchieveHueGoal;
import frc.robot.subsystems.GroupDisjointSequenceTest;
import frc.robot.subsystems.HistoryFSM;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MooreLikeFSM;
import frc.robot.subsystems.MooreLikeFSMMultiCommand;
import frc.robot.subsystems.RobotSignals;

public class RobotContainer {

  //FIXME options to select desired demonstrations
  private boolean allExamples                  = true;
  private boolean useAchieveHueGoal            = false || allExamples;
  private boolean useGroupDisjointSequenceTest = false || allExamples;
  private boolean useGroupDisjointParallelTest = false || allExamples;
  private boolean useTriggerNextCommand        = false || allExamples;
  private boolean useHistoryFSM                = false || allExamples;
  private boolean useIntake                    = false || allExamples;
  private boolean useMooreLikeFSM              = false || allExamples;
  private boolean useMooreLikeFSMMultiCommand  = false || allExamples;
  private boolean useStateMachine_FSM          = false || allExamples;
  private boolean useAutonomousSignal          = false || allExamples;
  private boolean useColorWheel                = false || allExamples;
  private boolean useMainDefault               = false || allExamples;
  private boolean useEnableDisable             = false || allExamples;

  //FIXME options for logging
  private boolean useConsole            = false;
  private boolean useDataLog            = true;
  private boolean useShuffleBoardLog    = false;

  // required classes and subsystems

  final int operatorControllerPort = 0; // user configurable port
  private final CommandXboxController m_operatorController = new CommandXboxController(operatorControllerPort);
  public CommandXboxController getM_operatorController() {
    return m_operatorController;
  }

  private final RobotSignals m_robotSignals = new RobotSignals(); // container and creator of all the LEDView subsystems
  public RobotSignals getM_robotSignals() {
    return m_robotSignals;
  }
  
  // optional classes and subsystems

  private Optional<AchieveHueGoal> m_achieveHueGoal = useAchieveHueGoal ? Optional.of(new AchieveHueGoal(m_robotSignals.m_achieveHueGoal)) : Optional.empty();
  public Optional<AchieveHueGoal> getM_achieveHueGoal() {
    return m_achieveHueGoal;
  }

  private Optional<GroupDisjointSequenceTest> m_groupDisjointSequenceTest = useGroupDisjointSequenceTest ? Optional.of(new GroupDisjointSequenceTest()) : Optional.empty(); // container and creator of all tests
  public Optional<GroupDisjointSequenceTest> getM_groupDisjointSequenceTest() {
    return m_groupDisjointSequenceTest;
  }

  private Optional<Boolean> m_groupDisjointParallelTest = useGroupDisjointParallelTest ? Optional.of(true) : Optional.empty();
  public Optional<Boolean> getM_groupDisjointParallelTest() {
    return m_groupDisjointParallelTest;
  }

  private Optional<Boolean> m_triggerNextCommand = useTriggerNextCommand ? Optional.of(true) : Optional.empty();
  public Optional<Boolean> getM_triggerNextCommand() {
    return m_triggerNextCommand;
  }

  private Optional<HistoryFSM> m_historyFSM = useHistoryFSM ? Optional.of(new HistoryFSM(m_robotSignals.m_historyDemo)) : Optional.empty();
  public Optional<HistoryFSM> getM_historyFSM() {
    return m_historyFSM;
  }

  private Optional<Intake> m_intake = useIntake ? Optional.of(new Intake(m_robotSignals.m_main)) : Optional.empty();
  public Optional<Intake> getM_intake() {
    return m_intake;
  }

  private Optional<MooreLikeFSM> m_mooreLikeFSM = useMooreLikeFSM ? Optional.of(new MooreLikeFSM(m_robotSignals.m_knightRider, 10.0, Color.kRed)) : Optional.empty();
  public Optional<MooreLikeFSM> getM_mooreLikeFSM() {
    return m_mooreLikeFSM;
  }

  private Optional<MooreLikeFSMMultiCommand> m_mooreLikeFSMMultiCommand = useMooreLikeFSMMultiCommand ? Optional.of(new MooreLikeFSMMultiCommand(m_robotSignals.m_imposter, 9.9, Color.kOrange)) : Optional.empty();
  public Optional<MooreLikeFSMMultiCommand> getM_mooreLikeFSMMultiCommand() {
    return m_mooreLikeFSMMultiCommand;
  }

  private Optional<Boolean> m_stateMachine = useStateMachine_FSM ? Optional.of(true) : Optional.empty();
  public Optional<Boolean> getM_stateMachine() {
    return m_stateMachine;
  }

  private Optional<Boolean> m_autonomousSignal = useAutonomousSignal ? Optional.of(true) : Optional.empty();
  public Optional<Boolean> getM_autonomousSignal() {
    return m_autonomousSignal;
  }

  private Optional<Boolean> m_colorWheel = useColorWheel ? Optional.of(true) : Optional.empty();
  public Optional<Boolean> getM_useColorWheel() {
    return m_colorWheel;
  }

  private Optional<Boolean> m_mainDefault = useMainDefault ? Optional.of(true) : Optional.empty();
  public Optional<Boolean> getM_useMainDefault() {
    return m_mainDefault;
  }

  private Optional<Boolean> m_enableDisable = useEnableDisable ? Optional.of(true) : Optional.empty();
  public Optional<Boolean> getM_useEnableDisable() {
    return m_enableDisable;
  }

  private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
  static
  {
    System.out.println("Loading: " + fullClassName);
    System.out.println("WPILib version " + edu.wpi.first.wpilibj.util.WPILibVersion.Version);
  }

  /**
   * Constructor creates most of the subsystems and operator controller bindings
   */
  @SuppressWarnings("resource")
  public RobotContainer() {

    if (!allExamples) new Alert("Not using allExamples", AlertType.kError).set(true);
 
    /* There are thousands of ways to do logging.
     * Here are 3 ways with options within the method.
     */
    configureCommandLogs(); // do early on otherwise log not ready for first commands
  }

  private CommandSchedulerLog schedulerLog;

  /**
   * Configure Command logging to Console/Terminal, DataLog, or ShuffleBoard
   */
  @SuppressWarnings("resource")
  public void configureCommandLogs()
  {
      if (useConsole || useDataLog || useShuffleBoardLog) {
        schedulerLog = new CommandSchedulerLog(useConsole, useDataLog, useShuffleBoardLog);
        schedulerLog.logCommandInitialize();
        schedulerLog.logCommandInterrupt();
        schedulerLog.logCommandFinish();
        schedulerLog.logCommandExecute();  // Can (optionally) generate a lot of output        
      }
      else {
        new Alert("No logging", AlertType.kWarning).set(true);
      }
  }

  /**
   * There are a variety of techniques to run I/O methods periodically and the example implemented
   * below in this code is a very simplistic start of a good possibility.
   * 
   * It demonstrates running before the scheduler loop to get a consistent set of sensor inputs.
   * After the scheduler loop completes all periodic outputs from subsystems are run such as data
   * logging and dashboards. (When enabled, the command scheduler runs its registered
   * subsystem.periodic() first but only for subsystems. Its use was threatened to be deprecated.)
   * (There is additional related discussion of periodic running in AchieveHueGoal.)
   *
   * There are clever ways to register classes say using a common "SubsystemTeam" class or
   * interface with a "register" method so they are automatically included in a list that can
   * easily be accessed with a loop. But this example is simplistic with no registration and no
   * loop - remember to type them in here and in any class that has multiple subsystems such as the 
   * example "GroupDisjointTest".
   * 
   * Security to prevent unauthorized running of periodic methods could be implemented in a variety
   * of ways but that error doesn't seem to happen so these examples have all "public" periodic
   * methods. Don't run them except in the designated places in the code.
   */

  /**
   * Run before commands and triggers from the Robot.periodic()
   *
   * <p>Run periodically before commands are run - read sensors, etc. Include all classes that have
   * periodic inputs or other need to run periodically.
   *
   */
  public void runBeforeCommands() {
    m_achieveHueGoal           .ifPresent((x)->x.runBeforeCommands());
    m_groupDisjointSequenceTest.ifPresent((x)->x.runBeforeCommands());
    m_historyFSM               .ifPresent((x)->x.runBeforeCommands());
    m_intake                   .ifPresent((x)->x.runBeforeCommands());
    m_mooreLikeFSM             .ifPresent((x)->x.runBeforeCommands());
    m_mooreLikeFSMMultiCommand .ifPresent((x)->x.runBeforeCommands());
    m_robotSignals                              .runBeforeCommands();
  }

  /**
   * Run after commands and triggers from the Robot.periodic()
   *
   * <p>Run periodically after commands are run - write logs, dashboards, indicators Include all
   * classes that have periodic outputs
   */
  public void runAfterCommands() {
    m_achieveHueGoal           .ifPresent((x)->x.runAfterCommands());
    m_groupDisjointSequenceTest.ifPresent((x)->x.runAfterCommands());
    m_historyFSM               .ifPresent((x)->x.runAfterCommands());
    m_intake                   .ifPresent((x)->x.runAfterCommands());
    m_mooreLikeFSM             .ifPresent((x)->x.runAfterCommands());
    m_mooreLikeFSMMultiCommand .ifPresent((x)->x.runAfterCommands());
    m_robotSignals                              .runAfterCommands();
  }
}
