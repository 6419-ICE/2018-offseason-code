package com.j0tech.utils.concurrent;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import org.usfirst.frc.team6419.robot.Robot;

/**
 * Lightweight lambda-based async. Tasks can be chained together using
 * {@link #andThen(AsyncTask, long)}, with a delay between them.
 *
 * If you don't want arguments, make T {@link Void}, then pass null to
 * {@link #execute(Consumer, Object)} as the second argument.
 * If you don't want a return type, make R {@link Void}, then return null from the last task in the
 * chain.
 * @param <T> argument for first task in chain
 * @param <R> return type
 */

@FunctionalInterface
public interface AsyncTask<T, R> {

    AtomicInteger runningAsyncTasks = new AtomicInteger(0);

    R run(T args);

    /**
     * Adds another task to be executed after a delay.
     * @param task task to be executed
     * @param delayMillis delay, in milliseconds
     * @param <V> return type of the new task
     * @return a new AsyncTask, consisting of the original task, a delay, and the task argument
     */
    default <V> AsyncTask<T, V> andThen(AsyncTask<R, V> task, long delayMillis) {
        Objects.requireNonNull(task);
        return (T t) -> {
            R r = run(t);
            try {
            	Thread.sleep(delayMillis);
            } catch (InterruptedException e) {
            	e.printStackTrace();
            }
            return task.run(r);
        };
    }

    /**
     * Executes the AsyncTask with a callback and an initial delay.
     * @param asyncCallback optional callback
     * @param args argument for first task in chain. If the first task has no arguments, args should
     *             be null.
     * @param initialDelayMillis time to delay before starting the task chain
     */
    default void execute(Consumer<R> asyncCallback, T args, long initialDelayMillis) {
        new Thread(() -> {
        	try {
        		Thread.sleep(initialDelayMillis);
        	} catch (InterruptedException e) {
        		e.printStackTrace();
        	}
            R result = run(args);
            runningAsyncTasks.decrementAndGet();
            if (asyncCallback != null) {
                asyncCallback.accept(result);
            }
        }, "Async Callback " + runningAsyncTasks.incrementAndGet()).start();
    }
    
    /**
     * Executes the AsyncTask with a callback.
     * @param asyncCallback callback
     * @param args argument for first task
     */
    default void execute(Consumer<R> asyncCallback, T args) {
    	execute(asyncCallback, args, 0);
    }
    
    /**
     * Executes the AsyncTask.
     * @param args argument for the first task
     */
    default void execute(T args) {
    	execute(null, args, 0);
    }
}