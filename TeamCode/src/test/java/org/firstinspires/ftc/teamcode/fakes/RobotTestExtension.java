package org.firstinspires.ftc.teamcode.fakes;

import org.junit.jupiter.api.extension.AfterEachCallback;
import org.junit.jupiter.api.extension.BeforeEachCallback;
import org.junit.jupiter.api.extension.ExtensionContext;
import org.junit.jupiter.api.extension.ParameterContext;
import org.junit.jupiter.api.extension.ParameterResolver;

/**
 * Lifecycle for {@link RobotTest}. The scheduler and subsystem registry are static, so they are
 * cleared before and after each test; {@code BeforeEachCallback} runs ahead of any {@code
 * @BeforeEach} in the test class, so subsystems constructed there see a clean registry.
 */
public final class RobotTestExtension
        implements BeforeEachCallback, AfterEachCallback, ParameterResolver {

    private static final ExtensionContext.Namespace NAMESPACE =
            ExtensionContext.Namespace.create(RobotTestExtension.class);

    @Override
    public void beforeEach(ExtensionContext context) {
        LoopRunner.reset();
    }

    @Override
    public void afterEach(ExtensionContext context) {
        LoopRunner.reset();
    }

    @Override
    public boolean supportsParameter(ParameterContext parameter, ExtensionContext context) {
        return parameter.getParameter().getType() == LoopRunner.class;
    }

    @Override
    public Object resolveParameter(ParameterContext parameter, ExtensionContext context) {
        return context.getStore(NAMESPACE)
                .getOrComputeIfAbsent(LoopRunner.class, key -> new LoopRunner(), LoopRunner.class);
    }
}
