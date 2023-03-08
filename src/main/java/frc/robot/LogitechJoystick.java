// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class LogitechJoystick {
    public static final int button3 = 3;
    public static final int button4 = 4;
    public static final int button5 = 5;
    public static final int button6 = 6;
    public static final int button7 = 7;
    public static final int button8 = 8;
    public static final int button9 = 9;
    public static final int button10 = 10;
    public static final int button11 = 11;
    public static final int button12 = 12;

    public static final int trigger = 1;
    public static final int thumbButton = 2;

    public static final int xAxis = 0;
    public static final int yAxis = 1;
    public static final int rotationAxis = 2;
    public static final int throttleAxis = 3;

    public static enum POVState
    {
        CENTER(-1),
        LEFT(270),
        RIGHT(90),
        UP(0),
        DOWN(180);

        private final int value;

        POVState(final int value)
        {
            this.value = value;
        }

        public int value() {return value;};
    }
}