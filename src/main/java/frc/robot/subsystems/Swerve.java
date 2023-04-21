package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

public class Swerve extends SubsystemBase {
  // private final ADIS16470_IMU gyro;
 // private final AHRS gyro;
  //private final CAN gyro= new CAN(1, 8, 4);
  //private final CANData gyroData = new CANData();

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  public static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  public ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);
  private Field2d field;

  public Swerve() {
    // gyro = new ADIS16470_IMU();
   // gyro = new AHRS();
    // gyro.configFactoryDefault();
    //zeroGyro();
    m_gyro.calibrate();
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
    //System.out.println("Current Heading: "+getYaw());
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
    }
  }

  public Pose2d getPose() {
    SmartDashboard.putNumber("pose X", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose Y", swerveOdometry.getPoseMeters().getY());

    SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());

    // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
    // about 14...0...360...346
    // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and
    // back leveling
    // 0-14, drive forward, 346-360 drive backward

   // SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
    //SmartDashboard.putNumber("gyro roll", gyro.getRoll());
    //SmartDashboard.putNumber("pitch rate", getPitchRate());

    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "position: module " + mod.moduleNumber, mod.getPosition().distanceMeters);
      SmartDashboard.putNumber(
          "angle: module " + mod.moduleNumber, mod.getPosition().angle.getDegrees());
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
   m_gyro.reset();
  }

 public Rotation2d getYaw() {
  return (Constants.Swerve.invertGyro)
  ? Rotation2d.fromDegrees(360 - m_gyro.getAngle())
  : Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  public double getYawRate() {
      //return gyro.getRawGyroZ();
      return m_gyro.getRate();
  }

  // public double getXFilteredAccelAngle() {
  //   return gyro.getXFilteredAccelAngle();
  // }

  // public double getYFilteredAccelAngle() {
  //   return gyro.getYFilteredAccelAngle();
  // }

  public double getPitch() {
    return 0.0;//gyro.getPitch();
  }

  public double getPitchRate() {
    return 0.0;//gyro.getRawGyroY();
  }

  public double getRoll() {
    return 0.0;//gyro.getRoll();
  }
  
  // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
  // about 14...0...360...346
  // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and back
  // leveling
  // 0-14, drive forward, 346-360 drive backward

  public void setX() {
    mSwerveMods[0].setAngleForX(45);
    mSwerveMods[1].setAngleForX(-45);
    mSwerveMods[2].setAngleForX(-45);
    mSwerveMods[3].setAngleForX(45);
  }
public void resetEncoders(){
  mSwerveMods[0].resetEncoder();
  mSwerveMods[1].resetEncoder();
  mSwerveMods[2].resetEncoder();
  mSwerveMods[3].resetEncoder();
}
  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
    //  SmartDashboard.putNumber(
      //    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
