// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Keybindings;
import frc.robot.Constants.Shooter;
import frc.robot.commands.ShootNote;
import frc.robot.commands.SwerveJoystickAuto;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // subsystems
  private final ElevatorSystem elevator = new ElevatorSystem(Elevator.M_ID_LEFT, Elevator.M_ID_RIGHT);
  private final ShooterSubsystem shooter = new ShooterSubsystem(Shooter.M_ID_LEFT, Shooter.M_ID_RIGHT);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // controllers
  private final XboxController controller = new XboxController(0);

  // motors
  private final Servo servo = new Servo(0);
  private boolean servoState = false;

  private int currentMax = 0;
  private ArrayList<Integer> maxes = new ArrayList<Integer>() {
    {
      add(0);
      add(50);
      add(100);
    }
  };

  // commands
  public RobotContainer() {
    // create named commands for pathplanner here
    NamedCommands.registerCommand("Drop", new ShootNote(shooter, () -> 0.1));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickAuto(
        swerveSubsystem,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> -controller.getRightY(),
        () -> -controller.getRightX()));

    shooter.setDefaultCommand(new ShootNote(shooter, () -> controller.getRightTriggerAxis()));

    configureBindings();
  }

  private enum BindingType {
    BUTTON,
    DPAD;
  }

  public Trigger createBinding(BindingType type, int button, Runnable trueAction) {
    return createBinding(type, button, trueAction, () -> {
    }, false);
  }

  public Trigger createBinding(BindingType type, int button, Runnable trueAction, Runnable falseAction) {
    return createBinding(type, button, trueAction, falseAction, false);
  }

  public Trigger createBinding(BindingType type, int button, Runnable trueAction, Runnable falseAction,
      boolean shouldRepeat) {
    switch (type) {
      case BUTTON:
        if (shouldRepeat) {
          return new JoystickButton(controller, button).whileTrue(new RepeatCommand(new InstantCommand(trueAction)))
              .onFalse(new InstantCommand(falseAction));
        } else {
          return new JoystickButton(controller, button).onTrue(new InstantCommand(trueAction))
              .onFalse(new InstantCommand(falseAction));
        }
      case DPAD:
        if (shouldRepeat) {
          return new POVButton(controller, button).whileTrue(new RepeatCommand(new InstantCommand(trueAction)))
              .onFalse(new InstantCommand(falseAction));
        } else {
          return new POVButton(controller, button).onTrue(new InstantCommand(trueAction))
              .onFalse(new InstantCommand(falseAction));
        }
    }
    return null;
  }

  // keybindings
  private void configureBindings() {

    createBinding(BindingType.BUTTON, Keybindings.BUTTON_A,
        () -> {
          servoState = !servoState;
          servo.setAngle(servoState ? 270 : -270);
        });

    createBinding(BindingType.BUTTON, Keybindings.BUTTON_B, () -> {
      currentMax = (currentMax + 1) % maxes.size();
      elevator.setEncoderMax(maxes.get(currentMax));
      System.out.println(maxes.get(currentMax));
    });

    createBinding(BindingType.BUTTON, Keybindings.BUTTON_Y, () -> {
      swerveSubsystem.zeroHeading();
    });

    // configure DPAD
    // elevator forward, backward -> start and stop
    createBinding(BindingType.DPAD, Keybindings.DPAD_UP, () -> elevator.setSpeed(Constants.Elevator.MOTOR_SPEED),
        () -> elevator.lock(), true);

    createBinding(BindingType.DPAD, Keybindings.DPAD_DOWN, () -> elevator.setSpeed(-Constants.Elevator.MOTOR_SPEED),
        () -> elevator.lock(), true);

    // left bumper -> swerve joystick
    new JoystickButton(controller, Keybindings.BUMPER_LEFT).whileTrue(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Drop It");
  }
}
