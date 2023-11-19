// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class RobotContainer {
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via  named factories in the {@link
   * CommandXboxController Xbox} class.
   */
  private void configureBindings() {
    //YOUR BINDINGS HERE

    m_driverController.a().onTrue(m_drivetrain.forwardSet());
    m_driverController.a().onFalse(m_drivetrain.forwardReset());
    m_driverController.b().onTrue(m_drivetrain.backwardSet());
    m_driverController.b().onFalse(m_drivetrain.backwardReset());
    m_driverController.leftBumper().onTrue(m_drivetrain.leftSet());
    m_driverController.leftBumper().onFalse(m_drivetrain.leftReset());
    m_driverController.rightBumper().onTrue(m_drivetrain.rightSet());
    m_driverController.rightBumper().onFalse(m_drivetrain.rightReset());

    m_driverController.x().onTrue(m_drivetrain.speedUp());
    m_driverController.y().onTrue(m_drivetrain.speedDown());

    m_driverController.leftStick().onTrue(
      m_drivetrain.forwardSet()
      .andThen(new WaitCommand(2))
      .andThen(m_drivetrain.forwardReset())
      .andThen(new WaitCommand(1))
      .andThen(m_drivetrain.backwardSet())
      .andThen(new WaitCommand(2))
      .andThen(m_drivetrain.backwardReset())
    );
    //m_driverController.b().whileTrue(m_drivetrain.exampleMethodCommand());
  }

}
