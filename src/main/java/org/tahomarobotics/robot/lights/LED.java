/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.lights;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;

// TODO: Fix so that changing collection mode while climbing or ready for l4 doesnt override those lights.
public class LED extends SubsystemIF {
    private static final LED INSTANCE = new LED();

    // -- Devices --

    private CANifier canifier = null;

    // -- Initialization --

    private LED() {
        if (RobotBase.isSimulation()) {
            return;
        }

        canifier = new CANifier(RobotMap.LED);
    }

    @Override
    public SubsystemIF initialize() {
        disable();
        return this;
    }

    public static LED getInstance() {
        return INSTANCE;
    }

    // -- COLORS --

    public void setColor(Color color) {
        if (canifier == null) { return; }

        // LED Channels
        //   - A = green
        //   - B = red
        //   - C = blue
        canifier.setLEDOutput(color.red, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(color.green, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(color.blue, CANifier.LEDChannel.LEDChannelC);
    }

    public void disable() {
        setColor(Color.kGold);
    }

    public void coral() {
        setColor(Color.kCoral);
    }

    public void algae() {
        setColor(new Color(0, 1, 0.25));
    }

    public void sync() {
        if (RobotState.isDisabled()) {
            disable();
            return;
        }

        if (Collector.getInstance().getCollectionMode() == GamePiece.CORAL) {
            coral();
        } else {
            algae();
        }
    }

    public void l4() {
        setColor(Color.kRed);
    }

    public void climb() {
        setColor(Color.kGreen);
    }
}