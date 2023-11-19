// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  public DriveSubsystem() {
    leftEncoderSim.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    rightEncoderSim.setDistancePerPulse(2 * Math.PI * 3 / 4096);

    SmartDashboard.putData(field);
  }
  
  private PWMSparkMax leftMotor = new PWMSparkMax(0);
  private PWMSparkMax rightMotor = new PWMSparkMax(1);

  private Encoder l_Encoder = new Encoder(0, 1);
  private Encoder r_Encoder = new Encoder(2, 3);
    
  private EncoderSim leftEncoderSim = new EncoderSim(l_Encoder);
  private EncoderSim rightEncoderSim = new EncoderSim(r_Encoder);

  private AnalogGyro gyro = new AnalogGyro(1);
  private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

  private DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
  KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
  KitbotGearing.k10p71,        // 10.71:1
  KitbotWheelSize.kSixInch,    // 6" diameter wheels.
  null           
 );
  
  private DifferentialDrive m_diffDrive  = new DifferentialDrive(leftMotor, rightMotor);

 DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  gyro.getRotation2d(),
  l_Encoder.getDistance(), r_Encoder.getDistance()
 );

  private Field2d field = new Field2d();


  @Override
  public void simulationPeriodic() {
      drivetrainSim.setInputs(leftMotor.get() * RobotController.getInputVoltage(), rightMotor.get() * RobotController.getInputVoltage());

      drivetrainSim.update(.02);

      leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
      leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
      rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
      rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
      gyroSim.setAngle(drivetrainSim.getHeading().getDegrees());

      var gyroAngle = gyro.getRotation2d();

      m_odometry.update(gyro.getRotation2d(),
                    leftEncoderSim.getDistance(),
                    rightEncoderSim.getDistance());
    field.setRobotPose(m_odometry.getPoseMeters());
     
  }

  public void forwardMove()
  {
    m_diffDrive.arcadeDrive(1, 0);
  }

  public InstantCommand fwrdCmd() {
    return new InstantCommand(() -> forwardMove());
  }

  public void backwardMove()
  {
    m_diffDrive.arcadeDrive(-1, 0);
  }

  public InstantCommand bwrdCmd() {
    return new InstantCommand(() -> backwardMove());
  }

  public void leftTurn()
  {
    m_diffDrive.arcadeDrive(0, 1);
  }

  public InstantCommand leftCmd() {
    return new InstantCommand(() -> leftTurn());
  }

  public void rightTurn()
  {
    m_diffDrive.arcadeDrive(0, -1);
  }

  public InstantCommand rightCmd() {
    return new InstantCommand(() -> rightTurn());
  }
}
