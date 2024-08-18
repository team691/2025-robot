// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Chuck;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveTrain m_robotDrive = new DriveTrain();
  private final Chuck m_output = new Chuck();
  private final Climber m_climber = new Climber();
  private final Lights m_lights = new Lights();

  // The driver's controller
  Joystick m_joystick1 = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
  XboxController m_operator = new XboxController(OIConstants.kDriverControllerPort3);

  // Initialize Sendable Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // TEST STAGE: Register PathFinder Commands

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Add PathPlanner autonomous
    m_chooser = AutoBuilder.buildAutoChooser();

    // Ignore controller warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_joystick1.getY()*setSpeed(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_joystick1.getX()*setSpeed(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_joystick2.getZ()*3, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_chooser.setDefaultOption("Wait", new WaitCommand(15));

    
    SmartDashboard.putData(m_chooser);
  }

  // Button mapping and config, pass to JoystickButton
  private void configureButtonBindings() {

            // This button for the DRIVER will stop the robot's drive
    new JoystickButton(m_joystick1, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

            // This button for the DRIVER will zero the gyro's angle
    new JoystickButton(m_joystick1, 3)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

            // This TRIGGER for the DRIVER  will accuate the Climber UP
    new JoystickButton(m_joystick2, 5)
        .toggleOnTrue(Commands.startEnd(
        () -> m_climber.AccuateUp(),
        () -> m_climber.AcctuateDown(),
        m_climber));

            // This button for the OPERATOR will intake the speaker motors
    new JoystickButton(m_operator,2)
        .onTrue(m_output.IntakeRing())
        .onFalse(m_output.stopRun());

            // This button for the OPERATOR will fire the release motor
    new JoystickButton(m_operator,3)
        .onTrue(m_output.Outake())
        .onFalse(m_output.stopRunAmp());
        
            // This button for the OPERATOR fires the upper speaker motor (prep)
    new JoystickButton(m_operator, 4)
        .onTrue(m_output.SpeakerShoot())
        .onFalse(m_output.stopRun());

            
            //Light function for OPERATOR lights speaker motor
    new JoystickButton(m_operator, 8)
        .toggleOnTrue(Commands.startEnd(
        () -> m_lights.ledPurple(),
        () -> m_lights.ledGreen(),
        m_lights));

            // Light function for OPERATOR lights amp motor
    new JoystickButton(m_operator, 7)
        .toggleOnTrue(Commands.startEnd(
        () -> m_lights.ledYellow(),
        () -> m_lights.ledGreen(),
        m_lights));
  }

    /* Not sure what this does 
    new JoystickButton(m_joystick1, 1)
        .onTrue(new RunCommand(
            () -> m_robotDrive.setLimit1()))
        .onFalse(new RunCommand(
            () -> m_robotDrive.unsettling()));
    } */

    //SPEED CMD
    public double setSpeed() {
        if (m_joystick1.getRawButton(1) == true) {
            return 3.0;//4
        }
        else {
            return 6.8;//7.8
        }
    }

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
