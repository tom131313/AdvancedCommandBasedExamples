package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import frc.robot.subsystems.AchieveHueGoal;
import frc.robot.subsystems.GroupDisjointSequenceTest;
import frc.robot.subsystems.HistoryFSM;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotSignals;
import frc.robot.subsystems.RobotSignals.LEDPatternSupplier;

public abstract class CommandsTriggers {
  private static CommandXboxController       m_operatorController;
  private static RobotSignals                m_robotSignals; // container and creator of all the LEDView subsystems
  private static Optional<AchieveHueGoal>    m_achieveHueGoal;
  private static Optional<GroupDisjointSequenceTest> m_groupDisjointSequenceTest;
  private static Optional<Boolean>           m_groupDisjointParallelTest;
  private static Optional<Boolean>           m_triggerNextCommand;
  private static Optional<HistoryFSM>        m_historyFSM;
  private static Optional<Intake>            m_intake;
  private static Optional<MooreLikeFSM>      m_UselightBar;
  private static Optional<Boolean>           m_UseAutonomousSignal;
  private static Optional<Boolean>           m_UseColorWheel;
  private static Optional<Boolean>           m_UseMainDefault;
  private static Optional<Boolean>           m_UseEnableDisable;
  
  private CommandsTriggers()
  {}

  public static void create(RobotContainer robotContainer)
  {
    m_operatorController = robotContainer.getM_operatorController();
    m_robotSignals = robotContainer.getM_robotSignals();
    m_achieveHueGoal = robotContainer.getM_achieveHueGoal();
    m_groupDisjointSequenceTest = robotContainer.getM_groupDisjointSequenceTest();
    m_groupDisjointParallelTest = robotContainer.getM_groupDisjointParallelTest();
    m_triggerNextCommand = robotContainer.getM_triggerNextCommand();
    m_historyFSM = robotContainer.getM_historyFSM();
    m_intake = robotContainer.getM_intake();
    m_UselightBar = robotContainer.getM_mooreLikeFSM();
    m_UseAutonomousSignal = robotContainer.getM_autonomousSignal();
    m_UseColorWheel = robotContainer.getM_useColorWheel();
    m_UseMainDefault = robotContainer.getM_useMainDefault();
    m_UseEnableDisable = robotContainer.getM_useEnableDisable();

    configureGameControllersBindings();
    configureTriggerBindings();
    configureDefaultCommands();
  }

  /**
   * Configure Suppliers
   */
  
  // produce a LED color pattern based on the timer current seconds of the minute
  private static RobotSignals.LEDPatternSupplier colorWheel =
      () ->
        LEDPattern.solid(
            Color.fromHSV(
                (int) (Timer.getFPGATimestamp() % 60.0 /* seconds of the minute */)
                    * 3 /* scale seconds to 180 hues per color wheel */,
                200,
                200));

  /**
   * Configure Commands
   */

  /**
   * Get disjointed sequence test from its creator for use by Robot - passing the reference up
   * 
   * @return Command to be scheduled to run disjointed sequence test
   */
  @SuppressWarnings("resource")
  public static Command getDisjointedSequenceTest() {
    if (m_groupDisjointSequenceTest.isPresent())
    {
      return m_groupDisjointSequenceTest.get().m_disjointedSequenceTest;
    }
    else
    {
      return runOnce(()-> new Alert("Group Disjointed Sequence Test not selected", AlertType.kWarning).set(true));
    }
  }

  /**
   * Get disjointed parallel test from its creator for use by Robot
   * 
   * @return Command to be scheduled to run disjointed parallel test
   */
  @SuppressWarnings("resource")
  public static Command getDisjointedParallelTest() {
    if (m_groupDisjointParallelTest.isPresent())
    {
      return
        // testing the TriggeredDisjointParallelGroup
        new TriggeredDisjointParallelGroup(true,
            Commands.print("immediately printed and waiting 4 seconds")
              .andThen(waitSeconds(4.))
              .andThen(print("ending - start next job in 1 second")),
            waitSeconds(6).andThen(Commands.print("at 6 of 6 seconds")),
            waitSeconds(5).andThen(Commands.print("at 5 of 6 seconds")),
            waitSeconds(4).andThen(Commands.print("at 4 of 6 seconds")),
            waitSeconds(1).andThen(Commands.print("at 1 of 6 seconds")),    
            waitSeconds(2).andThen(Commands.print("at 2 of 6 seconds")),
            waitSeconds(3).andThen(Commands.print("at 3 of 6 seconds"))   
          );      
    }
    else
    {
      return runOnce(()-> new Alert("Group Disjointed Parallel Test not selected", AlertType.kWarning).set(true));
    }
  }

  // Configuration of the trigger's action decorator must be executed somewhere. If the next
  // command in the action does not reference the trigger then it can be configured here.
  // If the action next command references the trigger, then set it in some initializing method
  // because of circular reference - trigger would reference the command and the command would
  // reference the trigger.
  // Even if the second command doesn't reference the trigger, an initializing method may be
  // advantageous to include logic not to set the trigger action, if some option is set not to use
  // it as in this example.
  private static final InternalButton firstJobTriggersSecond = new InternalButton(); // configure the action decorator must be executed somewhere
  // else and not here because of circular reference - trigger would reference the command and the command would reference the trigger

  private static final Command firstJob = runOnce(()->
          {
            firstJobTriggersSecond.setPressed(false); // add this - assuming not running a .whileTrue()
            System.out.println("first job running");
            firstJobTriggersSecond.setPressed(true); // add this - assuming next running with some "...True()"
          });

  private static final Command secondJob = print("second job running");

/**
   * Get first command triggers next command test
   * 
   * @return Command to be scheduled to run trigger next command test
   */
  @SuppressWarnings("resource")
  public static Command getFirstCommandTriggersNextTest() {
    if (m_triggerNextCommand.isPresent())
    {
      return firstJob;      
    }
    else
    {
      return runOnce(()-> new Alert("Trigger Next Command Test not selected", AlertType.kWarning).set(true));
    }
  }

  /**
   * Create a command to start the Moore FSM StateMachine Light Bar
   *
   * @return command that can be scheduled to start the Light Bar
   */
  @SuppressWarnings("resource")
  public static Command lightBar() {
    if (m_UselightBar.isPresent()) {
        // statements before the return are run early at initialization time
      return
          m_UselightBar.get().createLightBar()
          .withName("MooreLightBar");
    }
    else {
      return runOnce(()-> new Alert("Moore FSM Light Bar not selected", AlertType.kWarning).set(true));
    }
  }

/**
   * Create a command to signal Autonomous mode
   *
   * <p>Example of setting two signals by contrived example of composed commands
   *
   * @return LED pattern signal for autonomous mode
   */
  @SuppressWarnings("resource")
  public static Command setAutonomousSignal() {
    if (m_UseAutonomousSignal.isPresent()) {
      LEDPattern autoTopSignal =
            LEDPattern.solid(new Color(0.1, 0.2, 0.2))
            .blend(LEDPattern.solid(new Color(0.7, 0.2, 0.2)).blink(Seconds.of(0.1)));
            
      LEDPattern autoMainSignal = LEDPattern.solid(new Color(0.3, 1.0, 0.3));
      // statements before the return are run early at initialization time
      return
        parallel(
                // interrupting either of the two parallel commands with an external command interrupts
                // the group
                m_robotSignals.m_top.setSignal(autoTopSignal)
                    .withTimeout(6.0)/*.asProxy()*/ // timeout ends but the group continues and
                // the default command is not activated here with or without the ".andThen" command.
                // Use ".asProxy()" to disjoint the group and allow the "m_top" default command to run.
                // What happened to the ".andThen"? Beware using Proxy can cause surprising behavior!
                    .andThen(m_robotSignals.m_top.setSignal(autoTopSignal)),
                m_robotSignals.m_main.setSignal(autoMainSignal))
          .withName("AutoSignal");
    }
    else {
      return runOnce(()-> new Alert("Autonomous Signal not selected", AlertType.kWarning).set(true));
    }
  }

  /**
   * configure driver and operator controllers' buttons
   */
  private static void configureGameControllersBindings() {

    /**
     * Use operator "B" button for a fake indicator game piece is acquired
     */
    m_intake.ifPresent((x)->m_operatorController.b().whileTrue(x.gamePieceIsAcquired()));

    /**
     * Start History FSM Control with the operator "Y" button or it's time for a new color
     */
    m_historyFSM.ifPresent((x)->
    {
      var yButtonDebounceTime = Milliseconds.of(40.0);
      m_operatorController.y().debounce(yButtonDebounceTime.in(Seconds)).or(x::timesUp)
        .onTrue(x.newColor());
    });

    /**
     * Start a color wheel display with the operator "X" button
     */
    m_UseColorWheel.ifPresent((x) ->
    {
      var xButtonDebounceTime = Milliseconds.of(30.0);
      m_operatorController
          .x()
          .debounce(xButtonDebounceTime.in(Seconds), DebounceType.kBoth)
          .onTrue(m_robotSignals.m_top.setSignal(colorWheel));
    });

    /**
      * Goal setting demo control
      *
      * <p>The PID controller is not running initially until a setpoint is set by moving the operator
      * right trigger axis past the threshold at which time a command runs to achieve that goal.
      */
    m_achieveHueGoal.ifPresent((x)->
    {
      var triggerHueGoalDeadBand = 0.05; //triggers if past a small threshold (scale of 0 to 1)
      m_operatorController.rightTrigger(triggerHueGoalDeadBand)
          .onTrue(
              x.achieveHue( // goal-acceptance command
                  () -> m_operatorController.getRightTriggerAxis()*180.0 // supplying the setpoint
                  // scale joystick's 0 to 1 to computer color wheel hue 0 to 180
                  ));

      // immediately stop controller
      m_operatorController.a()
          .onTrue(x.interrupt());
    });
  }

  private static void configureTriggerBindings() {
    if (m_triggerNextCommand.isPresent()) { // no sense doing this if not requested
      firstJobTriggersSecond.onTrue(secondJob);
    }
  }

  /**
   * Configure some of the Default Commands
   *
   * <p>WARNING - heed the advice in the Robot.java comments about default commands
   */
  private static void configureDefaultCommands() {
    final LEDPattern topDefaultSignal = LEDPattern.solid(new Color(0.0, 0.0, 1.0));
    final LEDPattern mainDefaultSignal = LEDPattern.solid(new Color(0.0, 1.0, 1.0));
    final LEDPattern disabled = LEDPattern.solid(Color.kRed).breathe(Seconds.of(2.0));
    final LEDPattern enabled = LEDPattern.solid(Color.kGreen).breathe(Seconds.of(2.0));
    final LEDPatternSupplier enableDisableDefaultSignal =
        () -> DriverStation.isDisabled() ? disabled : enabled;
    // Intended that hue controller display always be ON so make it noticeable that it's OFF
    // since this default command should never run
    final LEDPattern hueControllerDisplayOffSignal = LEDPattern.solid(Color.kWhiteSmoke)
        .blink(Seconds.of(0.09));

    m_UseColorWheel.ifPresent((x)->
    {
      final Command topDefault =
          m_robotSignals
              .m_top
              .setSignal(topDefaultSignal)
              .ignoringDisable(true)
              .withName("TopDefault");
      m_robotSignals.m_top.setDefaultCommand(topDefault);
    });

    m_UseMainDefault.ifPresent((x)->
    {
      final Command mainDefault =
          m_robotSignals
              .m_main
              .setSignal(mainDefaultSignal)
              .ignoringDisable(true)
              .withName("MainDefault");
      m_robotSignals.m_main.setDefaultCommand(mainDefault);
    });

    m_UseEnableDisable.ifPresent((x)->
    {
      final Command enableDisableDefault =
          m_robotSignals
              .m_enableDisable
              .setSignal(enableDisableDefaultSignal)
              .ignoringDisable(true)
              .withName("EnableDisableDefault");
      m_robotSignals.m_enableDisable.setDefaultCommand(enableDisableDefault);
    });

    m_achieveHueGoal.ifPresent((x)->
    {
      final Command hueControllerDisplayDefault =
          m_robotSignals
              .m_achieveHueGoal
              .setSignal(hueControllerDisplayOffSignal)
              .ignoringDisable(true)
              .withName("HueControllerDisplayOff");
      m_robotSignals.m_achieveHueGoal.setDefaultCommand(hueControllerDisplayDefault);
      x.achieveHueDisplay().schedule(); // Not strictly the default but it should
      // always be running and we never see the default. Using the default command for the perpetual
      // command may be more robust as it restarts if cancelled for any reason but that wasn't used
      // in this example. cancelAll() will kill this so don't! (It wouldn't kill a default command.)
    });
  }
}
