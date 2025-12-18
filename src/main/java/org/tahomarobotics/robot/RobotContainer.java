/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.tahomarobotics.robot.util.FireShooterCommand;
import org.tahomarobotics.robot.util.PneumaticShootouts;
import org.tahomarobotics.robot.util.RetractCommand;

public class RobotContainer implements AutoCloseable {

    public final OI oi;

    public static class OI {

        private final XboxController controller = new XboxController(0);
        private final PneumaticShootouts shooter = new PneumaticShootouts();

        public OI() {
            configureBindings();
        }

        private void configureBindings() {
            new Trigger(() -> controller.getAButton())
                .onTrue(new FireShooterCommand(shooter));

            Trigger trigger = new Trigger(() -> controller.getBButton())
                .onTrue(new RetractCommand(shooter));
        }
    }

    public RobotContainer() {
        oi = new OI();
    }

    @Override
    public void close() {
    }
}
