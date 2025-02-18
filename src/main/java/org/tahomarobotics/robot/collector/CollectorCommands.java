package org.tahomarobotics.robot.collector;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.indexer.Indexer;

public class CollectorCommands {
    private static final Indexer indexer = Indexer.getInstance();
    static Command createZeroCommand(Collector collector) {
        return collector.runOnce(collector::setZeroingVoltage)
                        .andThen(Commands.waitSeconds(0.1))
                        .andThen(Commands.waitUntil(collector::isDeployStopped))
                        .withTimeout(CollectorConstants.DEPLOY_ZEROING_TIMEOUT)
                        .andThen(collector::zero)
                        .onlyIf(() -> collector.getTargetDeployState() == CollectorConstants.TargetDeployState.ZEROED);
    }

    public static Command createDeploymentControlCommand(Collector collector) {
        return collector.runOnce(() -> {
            if (collector.isDeploymentStowed()) {
                collector.deploymentTransitionToCollect();
            } else if (collector.isDeploymentCollecting()) {
                collector.deploymentTransitionToStow();
            }
        });
    }

    /** @return On true and on false commands. */
    public static Pair<Command, Command> createCollectorControlCommands(Collector collector) {
        Command onTrue = collector.runOnce(() -> {
            if (collector.isDeploymentCollecting() && !collector.isHoldingAlgae() /*&& !indexer.beamBreakTripped()*/) {
                // TODO: Only if not coral collected (from indexer or arm).
                collector.collectorTransitionToCollecting();
            }
        });
        Command onFalse = collector.runOnce(() -> {
            if (collector.getTargetCollectorState() != CollectorConstants.TargetCollectorState.HOLDING_ALGAE) {
                collector.collectorTransitionToDisabled();
            }
        });

        return Pair.of(onTrue, onFalse);
    }

    /** @return On true and on false commands. */
    public static Pair<Command, Command> createEjectCommands(Collector collector) {
        Command onTrue = collector.runOnce(collector::transitionToEjecting);
        Command onFalse = collector.runOnce(collector::cancelEjecting);

        return Pair.of(onTrue, onFalse);
    }
}
