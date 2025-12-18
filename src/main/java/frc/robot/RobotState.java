package frc.robot;

import java.util.HashMap;

import frc.robot.Drive.Drive;
import frc.robot.Drive.Drive.DriveProfiles;
import frc.robot.util.SubsystemProfiles;

public class RobotState {

    public enum RobotAction {
        kAutoDefault,
        kTeleopDefault
    }

    private Drive m_drive;
    private SubsystemProfiles<RobotAction> m_profiles;
    private static RobotState m_instance;

    public RobotState(Drive drive) {
        m_drive = drive;

        HashMap<RobotAction, Runnable> hash = new HashMap<>();
        hash.put(RobotAction.kAutoDefault, this::autoPeriodic());
        hash.put(RobotAction.kTeleopDefault, () -> {});

        m_profiles = new SubsystemProfiles<>(hash, RobotAction.kTeleopDefault);
    }

    public void updateRobotAction(RobotAction action) {
        DriveProfiles newDriveState = DriveProfiles.kTeleopDefault;

        switch (action) {
            case kAutoDefault:
                newDriveState = DriveProfiles.kAutoDefault;
                break;
            case kTeleopDefault:
                newDriveState = DriveProfiles.kTeleopDefault;
                break;
            default:
                break;
        }

        if (newDriveState != m_drive.getCurrentProfile()) {
            m_drive.updateProfile(newDriveState);
        }

        m_profiles.setCurrentProfile(action);
    }

    public static RobotState getInstance() {
        return m_instance;
    }
    
    public static RobotState startInstance(
        Drive drive) {
    if (m_instance == null) {
        m_instance = new RobotState(drive);
    }
    return m_instance;
    }

    public RobotAction getCurrAction() {
    return m_profiles.getCurrentProfile();
    }

    public void autoPeriodic() {
        // IDK what to put here
    }
}
