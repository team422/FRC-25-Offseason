package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.custom.Custom;
import frc.robot.subsystems.custom.CustomIOKraken;
import frc.robot.subsystems.custom.CustomIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Custom m_custom;

  private DriverControls m_driverControls;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureButtonBindings();
  }

  /** Configure the subsystems. */
  private void configureSubsystems() {
    if (RobotBase.isReal()) {
      m_custom = new Custom(new CustomIOKraken(Ports.kCustomMotor, Ports.kMainCanivoreName));
    } else {
      m_custom = new Custom(new CustomIOSim());
    }
  }

  /** Configure the commands. */
  private void configureCommands() {
    // Auto commands

    // we start here so autofactory won't be null
    RobotState.startInstance(m_custom);
  }

  /** Configure the controllers. */
  private void configureControllers() {
    // m_driverControls = new DriverControlsXbox(0);
    m_driverControls = new DriverControlsPS5(0);
  }

  /** Configure the button bindings. */
  private void configureButtonBindings() {
    m_driverControls
        .velocityControl()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotState.getInstance().getAction() == RobotAction.kVelocityControl) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kDefault);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kVelocityControl);
                  }
                }));

    m_driverControls
        .voltageControl()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotState.getInstance().getAction() == RobotAction.kVoltageControl) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kDefault);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kVoltageControl);
                  }
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public String getSelectedAuto() {
    return m_autoChooser.getSendableChooser().getSelected();
  }
}
