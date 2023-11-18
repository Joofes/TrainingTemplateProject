// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  public DriveSubsystem() {
    leftEncoderSim.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    rightEncoderSim.setDistancePerPulse(2 * Math.PI * 3 / 4096);
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
  }
}
