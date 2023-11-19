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

  boolean forward; 
  boolean backward;
  boolean left;
  boolean right;

  float movementSpeed = 1; //scaled between 0-1 and multiplied by -1/1 in movement handlers
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
     if (forward){
      forwardMove();
     }
     if (backward)
     {
       backwardMove();;
     }
     if(left)
     {
       leftTurn();
     }
     if(right)
     {
       rightTurn();
     }
  }
  void movementChanges(int type, boolean set)
  {
    if(type == 1)
    {
      forward = set;
    }
    if(type == 2)
    {
      backward = set;
    }
    if(type == 3)
    {
      left = set;
    }
    if(type == 4)
    {
      right = set;
    }

  }

  public void forwardMove()
  {
    m_diffDrive.arcadeDrive(1 * movementSpeed, 0);
  }

  public InstantCommand forwardSet() {
    return new InstantCommand(() -> movementChanges(1, true));
  }

  public InstantCommand forwardReset() {
    return new InstantCommand(() -> movementChanges(1, false));
  }

  public void backwardMove()
  {
    m_diffDrive.arcadeDrive(-1 * movementSpeed, 0);
  }

  public InstantCommand backwardSet() {
    return new InstantCommand(() -> movementChanges(2, true));
  }

  public InstantCommand backwardReset() {
    return new InstantCommand(() -> movementChanges(2, false));
  }

  public void leftTurn()
  {
    m_diffDrive.arcadeDrive(0, -.5);
  }

  public InstantCommand leftSet() {
    return new InstantCommand(() -> movementChanges(3, true));
  }

  public InstantCommand leftReset() {
    return new InstantCommand(() -> movementChanges(3, false));
  }


  public void rightTurn()
  {
    m_diffDrive.arcadeDrive(0, .5);
  }

  public InstantCommand rightSet() {
    return new InstantCommand(() -> movementChanges(4, true));
  }

  public InstantCommand rightReset() {
    return new InstantCommand(() -> movementChanges(4, false));
  }

  public void speedHandler(double change)
  {
    movementSpeed += change;
    if(movementSpeed < 0)
    {movementSpeed = 0;}
    if(movementSpeed > 1)
    {movementSpeed = 1;}

    SmartDashboard.putNumber("speed", movementSpeed);
  }

  public InstantCommand speedUp()
  {
    return new InstantCommand(() -> speedHandler(0.25));
  }

  public InstantCommand speedDown()
  {
    return new InstantCommand(() -> speedHandler(-0.25));
  }
}
