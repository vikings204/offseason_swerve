package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.util.Math204;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants204;;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;
  
  private double turningPDeg = 0;
  private int turningPQuad = 0; // top left is 1 counterclockwise
  private double turningTotalDeg = 0.0;
  
  public final TalonSRX angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  //private final TalonSRXFeedbackDevice angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
   // angleEncoder = new CANCoder(moduleConstants.cancoderID);
    //configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new TalonSRX(moduleConstants.angleMotorID);
    angleMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0); 

    //turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0); //Set the feedback device that is hooked up to the talon

    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    
    //desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    //double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    //integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
   // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    //angleMotor.configFactoryDefault();
   // CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.configContinuousCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    //angleMotor.
    angleMotor.setInverted(false);
    angleMotor.setNeutralMode(NeutralMode.Brake);
   // integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleMotor.config_kP(0, 5.0);
    angleMotor.config_kI(0, .001);
    angleMotor.config_kD(0, 0);
    angleMotor.config_kF(0, Constants.Swerve.angleKFF);
    //angleController.setFF(Constants.Swerve.angleKFF);
    angleMotor.configVoltageCompSaturation(Constants.Swerve.voltageComp);
    //angleMotor.voltageComp(Constants.Swerve.voltageComp);
    //angleMotor.burnFlash();
    //Timer.delay(1);
    //resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.angleKP);
    driveController.setI(Constants.Swerve.angleKI);
    driveController.setD(Constants.Swerve.angleKD);
    driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
            SmartDashboard.putNumber("Angle Position Setting Mod" + moduleNumber, angle.getDegrees());
            SmartDashboard.putNumber("Encoder Position Setting without Offset" + moduleNumber, (((angle.getDegrees())/360)*1023));   
            SmartDashboard.putNumber("Encoder Position Setting with Offset" + moduleNumber, (((angle.getDegrees()+angleOffset.getDegrees())/360)*1023));         
    //angleMotor.set(TalonSRXControlMode.Position, (((angle.getDegrees()+angleOffset.getDegrees())/360)*1023));

    
    double setter = SwerveContinuous(angle.getDegrees());
    angleMotor.set(TalonSRXControlMode.Position, (((setter+angleOffset.getDegrees())/360)*1023));
  

    lastAngle = angle;
  }

  public void setAngleForX(double angle) {
    driveMotor.set(0);
    angleMotor.set(TalonSRXControlMode.Position, (angle/360)*1023);
    //angleController.setReference(angle, ControlType.kPosition);
  }

  private Rotation2d getAngle() {
    SmartDashboard.putNumber("getAngleCall position Mod" + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees());
    //System.out.println("Encoder Position Mod "+moduleNumber+": "+(angleMotor.getSelectedSensorPosition()/1023)*360);
    return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
  }

  public Rotation2d getCanCoder() {
   // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
   return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }
public void resetEncoder(){
  angleEncoder.setPosition(0);
}
  public SwerveModulePosition getPosition() {
    SmartDashboard.putNumber("Raw Angle Reading " + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360);
    SmartDashboard.putNumber("angleEncoderCurrent Reading " + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360- angleOffset.getDegrees());
    //System.out.println("Encoder Position: "+((angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees()));
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromDegrees((angleMotor.getSelectedSensorPosition()/1023)*360- angleOffset.getDegrees()));
       
  }
  private double SwerveContinuous(double cDeg) {
    double nDeg;
    int cQuad = Math204.GetQuadrant(cDeg);
    if ((turningPQuad == 3 || turningPQuad == 4) && cQuad == 1) {
        nDeg = (360-turningPDeg)+cDeg;
    } else if ((turningPQuad == 1 || turningPQuad == 2) && cQuad == 4) {
        nDeg = -((360-cDeg)+turningPDeg);
    } else {
        nDeg = cDeg - turningPDeg;
    }

    turningTotalDeg += nDeg;
    turningPDeg = cDeg;
    turningPQuad = cQuad;
    return turningTotalDeg;
}
}
