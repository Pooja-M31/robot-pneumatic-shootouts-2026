package org.tahomarobotics.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.climber.Climber;
import org.tahomarobotics.robot.climber.ClimberConstants;

public class ClimberCommands {
    private static final Climber climber = Climber.getInstance();

    public static Command getClimberCommand() {
         return Commands.deferredProxy(() -> switch (climber.getClimbState()) {
             case STOWED -> Commands.runOnce(climber::deploy);
             case DEPLOYED -> Commands.runOnce(climber::climb)
                                      .andThen(Commands.runOnce(climber::deploySolenoid))
                                      .andThen(Commands.waitUntil(climber::isAtTargetPosition))
                                      .andThen(Commands.runOnce(climber::disableClimberMotors))
                                      .andThen(Commands.waitSeconds(ClimberConstants.RATCHET_SOLENOID_DEPLOY_TIME))
                                      .andThen(Commands.runOnce(climber::disableSolenoid));
             case CLIMBED -> Commands.runOnce(climber::stow);
             }
         );
    }
}
