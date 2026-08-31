package org.firstinspires.ftc.teamcode.fakes;

import org.junit.jupiter.api.extension.ExtendWith;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a subsystem test. Resets the command scheduler and subsystem registry around every test and
 * supplies a fresh {@link LoopRunner} as a test parameter.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
@ExtendWith(RobotTestExtension.class)
public @interface RobotTest {}
