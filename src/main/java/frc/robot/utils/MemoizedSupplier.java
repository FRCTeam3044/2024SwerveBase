package frc.robot.utils;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

/**
 * Memoized version of {@link Supplier}. Useful for things like generating a
 * trajectory once at the start of a command.
 * 
 * @param <T>
 * @param delegate
 * @return
 */
public class MemoizedSupplier<T> {
    private final Supplier<T> delegate;
    private final AtomicReference<T> value = new AtomicReference<>();

    /**
     * Create a new memoized supplier
     * 
     * @param delegate The supplier to memoize
     */
    public MemoizedSupplier(Supplier<T> delegate) {
        this.delegate = delegate;
    }

    /**
     * Get the memoized value, computing it if necessary
     * 
     * @return The memoized value
     */
    public T get() {
        T val = value.get();
        if (val == null) {
            val = value.updateAndGet(cur -> cur == null ? delegate.get() : cur);
        }
        return val;
    }

    /**
     * Clear the memoized value, forcing the next call to {@link #get()} to
     * recompute
     */
    public void clear() {
        value.set(null);
    }

    public static <T> MemoizedSupplier<T> memoize(Supplier<T> delegate) {
        return new MemoizedSupplier<>(delegate);
    }
}
