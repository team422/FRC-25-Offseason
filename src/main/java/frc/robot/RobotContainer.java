package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.turret.flywheel.FlywheelIOSim;
import frc.robot.subsystems.turret.hood.HoodIOKraken;
import frc.robot.subsystems.turret.hood.HoodIOSim;
import frc.robot.subsystems.turret.pivot.PivotIOKraken;
import frc.robot.subsystems.turret.pivot.PivotIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive m_drive;
  private Turret m_turret;

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
    if (Constants.kCurrentMode == Mode.REAL) {
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(0),
              new ModuleIOTalonFX(1),
              new ModuleIOTalonFX(2),
              new ModuleIOTalonFX(3));
      m_turret =
          new Turret(
              new PivotIOKraken(Ports.kPivot, Ports.kMainCanivoreName),
              new FlywheelIOKraken(
                  Ports.kTopShooter, Ports.kBottomShooter, Ports.kMainCanivoreName),
              new HoodIOKraken(Ports.kHood, Ports.kMainCanivoreName));
    } else if (Constants.kCurrentMode == Mode.SIM) {
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
      m_turret = new Turret(new PivotIOSim(), new FlywheelIOSim(), new HoodIOSim());
    }
  }

  /** Configure the commands. */
  private void configureCommands() {
    // Auto commands

    // we start here so autofactory won't be null
    RobotState.startInstance(m_drive, m_turret);
  }

  /** Configure the controllers. */
  private void configureControllers() {
    // m_driverControls = new DriverControlsXbox(0);
    m_driverControls = new DriverControlsPS5(0);
  }

  /** Configure the button bindings. */
  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            m_driverControls::getForward,
            m_driverControls::getStrafe,
            m_driverControls::getTurn,
            false));

    m_driverControls
        .autoAim()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotState.getInstance().getAction() != RobotAction.kAutoAim) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kAutoAim);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kDefault);
                  }
                }));

    m_driverControls
        .autoShoot()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotState.getInstance().getAction() != RobotAction.kAutoShooting) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kAutoShooting);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kDefault);
                  }
                }));

    m_driverControls
        .defaultShoot()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotState.getInstance().getAction() != RobotAction.kDefaultShooting) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kDefaultShooting);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kDefault);
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
