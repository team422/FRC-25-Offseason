package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.lib.littletonUtils.AllianceFlipUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class SetpointGenerator {
  private static final Translation2d m_speakerPose =
      AllianceFlipUtil.apply(FieldConstants.kSpeakerLocation);

  private static final InterpolatingDoubleTreeMap m_topVelocityTree =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_bottomVelocityTree =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodTree = new InterpolatingDoubleTreeMap();

  static {
    m_topVelocityTree.put(1., 1.);
    m_topVelocityTree.put(200., 200.);
    m_bottomVelocityTree.put(1., 1.);
    m_bottomVelocityTree.put(200., 200.);
    hoodTree.put(1., 1.);
    hoodTree.put(200., 130.);
  }

  public static Rotation2d getShooterPosition(Drive drive) {
    var currPose = drive.getPose();
    double x = m_speakerPose.getX() - currPose.getX();
    double y = m_speakerPose.getY() - currPose.getY();

    var desiredHeading = Rotation2d.fromRadians(Math.atan2(y, x)).minus(drive.getRotation());
    Logger.recordOutput("Turret/angle", desiredHeading.plus(drive.getRotation()));

    return desiredHeading;
  }

  public static Rotation2d getHoodPosition(Drive drive) {
    var currTranslation = drive.getPose().getTranslation();
    double distance = m_speakerPose.getDistance(currTranslation);
    return Rotation2d.fromDegrees(hoodTree.get(distance));
  }

  public static Pair<Double, Double> getShooterSpeeds(Drive drive) {
    var currTranslation = drive.getPose().getTranslation();
    double distance = m_speakerPose.getDistance(currTranslation);

    return Pair.of(m_topVelocityTree.get(distance), m_bottomVelocityTree.get(distance));
  }
}
