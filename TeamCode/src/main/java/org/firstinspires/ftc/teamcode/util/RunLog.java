package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Machine-parseable run log: one directory per run under {@code /sdcard/FIRST/logs/}, holding
 * {@code meta.json}, a per-loop {@code signals.csv} and an {@code events.jsonl} stream. Retrieve
 * with {@code adb pull /sdcard/FIRST/logs/}.
 *
 * <p>{@link #writeLoop(int)} samples on the calling thread and hands bytes to a bounded queue, so
 * it never blocks the control loop; a full queue drops the row and bumps {@link #dropped()}. All
 * failures degrade to a disabled sink and a Driver Station warning: no method here throws.
 *
 * <p>{@code t_s} and event {@code t} are seconds since open from {@code System.nanoTime()}. Wall
 * clock appears only as {@code startedWallClockMs} in {@code meta.json} and may be wrong on a hub
 * with no Driver Station connected.
 */
public final class RunLog implements AutoCloseable {

    /** Run directories kept in the log folder, oldest deleted first. */
    public static final int MAX_RUNS = 25;

    /** Bytes written before the log truncates itself. */
    public static final long MAX_BYTES = 32L * 1024 * 1024;

    private static final int QUEUE_CAPACITY = 2048;
    private static final long POLL_MS = 100;
    private static final long FLUSH_INTERVAL_MS = 2000;
    /** Graceful drain, then a shorter window after interrupting. The stop budget is about 900 ms. */
    private static final long STOP_TIMEOUT_MS = 600;

    private static final long FORCE_TIMEOUT_MS = 200;
    private static final int SIGNAL_DECIMALS = 4;
    private static final int TIME_DECIMALS = 3;
    private static final Charset UTF_8 = Charset.forName("UTF-8");

    private static final int SIGNALS = 0;
    private static final int EVENTS = 1;
    private static final int META = 2;

    /** One queued line and the file it belongs to. */
    private static final class Entry {
        final int dest;
        final byte[] data;

        Entry(int dest, byte[] data) {
            this.dest = dest;
            this.data = data;
        }
    }

    /** Appends the current value of one column, or nothing if the source throws. */
    private interface Sampler {
        void append(StringBuilder out);
    }

    private final boolean enabled;
    private final File dir;
    private final String opModeName;
    private final long startNanos;
    private final long startedWallClockMs;

    private final FileOutputStream signalsFile;
    private final FileOutputStream eventsFile;
    private final OutputStream signals;
    private final OutputStream events;

    private final List<String> columns = new ArrayList<>();
    private final List<Sampler> samplers = new ArrayList<>();
    private final StringBuilder row = new StringBuilder(256);

    private final ArrayBlockingQueue<Entry> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    private final AtomicInteger dropped = new AtomicInteger();
    private final AtomicLong bytesWritten = new AtomicLong();
    private final ExecutorService writer;

    private boolean frozen;
    private int rows;
    private boolean closed;
    private volatile boolean stopping;
    private volatile boolean capped;
    private boolean warned;
    private boolean closeInterrupted;

    /** Log folder on internal storage. Must be under {@link AppUtil#FIRST_FOLDER} to be writable. */
    public static RunLog openDefault(String opModeName) {
        File dir = new File(AppUtil.FIRST_FOLDER, "logs");
        try {
            AppUtil.getInstance().ensureDirectoryExists(dir);
        } catch (RuntimeException e) {
            return degrade("cannot create " + dir + ": " + e);
        }
        return open(dir, opModeName);
    }

    /** Opens a run directory under {@code parentDir}. Returns {@link #disabled()} on any failure. */
    public static RunLog open(File parentDir, String opModeName) {
        String name = opModeName == null ? "" : opModeName;
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        File dir = new File(parentDir, stamp + "-" + sanitize(name));
        FileOutputStream signalsFile = null;
        FileOutputStream eventsFile = null;
        try {
            if (!dir.isDirectory() && !dir.mkdirs()) {
                throw new IOException("cannot create " + dir);
            }
            signalsFile = new FileOutputStream(new File(dir, "signals.csv"));
            eventsFile = new FileOutputStream(new File(dir, "events.jsonl"));
            RunLog log = new RunLog(dir, name, signalsFile, eventsFile);
            prune(parentDir, dir);
            log.start();
            return log;
        } catch (IOException | RuntimeException e) {
            closeQuietly(signalsFile);
            closeQuietly(eventsFile);
            return degrade(String.valueOf(e));
        }
    }

    /** Sink that accepts every call and writes nothing. */
    public static RunLog disabled() {
        return new RunLog();
    }

    private RunLog() {
        enabled = false;
        dir = null;
        opModeName = "";
        startNanos = 0;
        startedWallClockMs = 0;
        signalsFile = null;
        eventsFile = null;
        signals = null;
        events = null;
        writer = null;
    }

    private RunLog(File dir, String opModeName, FileOutputStream signals, FileOutputStream events) {
        enabled = true;
        this.dir = dir;
        this.opModeName = opModeName;
        startNanos = System.nanoTime();
        startedWallClockMs = System.currentTimeMillis();
        signalsFile = signals;
        eventsFile = events;
        this.signals = new BufferedOutputStream(signals, 8192);
        this.events = new BufferedOutputStream(events, 8192);
        writer = Executors.newSingleThreadExecutor(RunLog::newWriterThread);
    }

    private void start() {
        writer.execute(this::writerLoop);
        StringBuilder line = openEvent("run", "start");
        line.append(",\"opMode\":\"");
        appendEscaped(line, opModeName);
        line.append("\"}\n");
        enqueue(EVENTS, line);
    }

    // ------------------------------------------------------------------ signals

    /** Registers a numeric column. Must be called before the first {@link #writeLoop(int)}. */
    public void addSignal(String name, DoubleSupplier value) {
        addColumn(name, out -> appendFixed(out, value.getAsDouble(), SIGNAL_DECIMALS));
    }

    /** Registers a boolean column, written as {@code 1} or {@code 0}. */
    public void addFlag(String name, BooleanSupplier value) {
        addColumn(name, out -> out.append(value.getAsBoolean() ? '1' : '0'));
    }

    /**
     * Registers a numeric column sampled at most every {@code intervalMs}, repeating the cached
     * value in between. For sources that cost a hardware round-trip, such as battery voltage.
     */
    public void addSlowSignal(String name, DoubleSupplier value, long intervalMs) {
        addColumn(name, new Sampler() {
            private long lastMs = Long.MIN_VALUE;
            private double cached;
            private boolean present;

            @Override
            public void append(StringBuilder out) {
                long nowMs = elapsedMs();
                if (!present || nowMs - lastMs >= intervalMs) {
                    cached = value.getAsDouble();
                    present = true;
                    lastMs = nowMs;
                }
                appendFixed(out, cached, SIGNAL_DECIMALS);
            }
        });
    }

    private void addColumn(String name, Sampler sampler) {
        if (!enabled) {
            return;
        }
        if (frozen) {
            throw new IllegalStateException("signals are frozen after the first row");
        }
        if (columns.contains(name)) {
            throw new IllegalArgumentException("duplicate signal name: " + name);
        }
        columns.add(name);
        samplers.add(sampler);
    }

    // ------------------------------------------------------------------ writing

    /** Samples every registered signal into one CSV row. Never blocks; may drop the row. */
    public void writeLoop(int loopNumber) {
        if (!enabled || closed || capped) {
            return;
        }
        if (!frozen) {
            frozen = true;
            enqueue(META, buildMeta());
            StringBuilder header = new StringBuilder(128).append("t_s,loop");
            for (int i = 0; i < columns.size(); i++) {
                header.append(',').append(columns.get(i));
            }
            header.append('\n');
            enqueue(SIGNALS, header);
        }
        row.setLength(0);
        appendFixed(row, elapsedSeconds(), TIME_DECIMALS);
        row.append(',').append(loopNumber);
        for (int i = 0; i < samplers.size(); i++) {
            row.append(',');
            try {
                samplers.get(i).append(row);
            } catch (RuntimeException ignored) {
                // A broken supplier leaves the field empty rather than killing the log.
            }
        }
        row.append('\n');
        rows++;
        enqueue(SIGNALS, row);
    }

    /** Writes one free-form event, for example {@code event("note", "driver took over")}. */
    public void event(String type, String detail) {
        if (!enabled || closed || capped) {
            return;
        }
        StringBuilder line = new StringBuilder(96).append('{').append("\"t\":");
        appendFixed(line, elapsedSeconds(), TIME_DECIMALS);
        line.append(",\"type\":\"");
        appendEscaped(line, type);
        line.append('"');
        if (detail != null) {
            line.append(",\"detail\":\"");
            appendEscaped(line, detail);
            line.append('"');
        }
        line.append("}\n");
        enqueue(EVENTS, line);
    }

    /** Writes a command lifecycle event: {@code start}, {@code interrupt} or {@code finish}. */
    public void commandEvent(String event, String name, int id) {
        if (!enabled || closed || capped) {
            return;
        }
        StringBuilder line = openEvent("command", event);
        line.append(",\"name\":\"");
        appendEscaped(line, name);
        line.append("\",\"id\":").append(id).append("}\n");
        enqueue(EVENTS, line);
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Rows and events discarded because the writer fell behind. */
    public int dropped() {
        return dropped.get();
    }

    /** Run directory, or null when disabled. */
    public File directory() {
        return dir;
    }

    /** Flushes, fsyncs and stops the writer. Idempotent; safe inside the OpMode stop budget. */
    @Override
    public void close() {
        if (!enabled || closed) {
            return;
        }
        closed = true;
        if (!capped) {
            StringBuilder line = openEvent("run", "end");
            line.append(",\"loops\":")
                    .append(rows)
                    .append(",\"dropped\":")
                    .append(dropped.get())
                    .append(",\"bytes\":")
                    .append(bytesWritten.get())
                    .append("}\n");
            offer(new Entry(EVENTS, bytes(line)));
        }
        if (!frozen) {
            enqueue(META, buildMeta());
        }
        // The OpMode thread is interrupted on stop, and awaitTermination throws immediately when
        // the flag is already set, so the waits below would not wait at all. Clear it for the
        // duration and restore it on the way out.
        boolean wasInterrupted = Thread.interrupted();
        try {
            stopping = true;
            writer.shutdown();
            if (!awaitWriter(STOP_TIMEOUT_MS)) {
                writer.shutdownNow();
                if (!awaitWriter(FORCE_TIMEOUT_MS)) {
                    // The writer may still be inside a write, so closing the streams under it
                    // would corrupt the tail rather than save it.
                    RobotLog.addGlobalWarningMessage("RunLog writer did not stop; log tail lost");
                    return;
                }
            }
            syncAndClose(signals, signalsFile);
            syncAndClose(events, eventsFile);
        } finally {
            if (wasInterrupted || closeInterrupted) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /** Waits for the writer. Records a fresh interrupt rather than re-arming it mid-close. */
    private boolean awaitWriter(long timeoutMs) {
        try {
            return writer.awaitTermination(timeoutMs, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            closeInterrupted = true;
            return false;
        }
    }

    // ------------------------------------------------------------------ writer thread

    /**
     * Named daemon writer thread. Not {@code ThreadPool.newSingleThreadExecutor}: its
     * {@code RecordingThreadPool} reads {@code android.util.LongSparseArray} on every
     * {@code execute}, which cannot run in an off-robot unit test. Daemon so a writer that outlives
     * {@link #close()} cannot keep the app alive.
     */
    private static Thread newWriterThread(Runnable body) {
        Thread thread = new Thread(body, "RunLogWriter");
        thread.setDaemon(true);
        return thread;
    }

    private void writerLoop() {
        long lastFlushNanos = System.nanoTime();
        boolean interrupted = false;
        while (true) {
            Entry entry;
            if (stopping || interrupted) {
                // Nothing more is enqueued once stopping is set, so drain without waiting.
                entry = queue.poll();
            } else {
                try {
                    entry = queue.poll(POLL_MS, TimeUnit.MILLISECONDS);
                } catch (InterruptedException e) {
                    interrupted = true;
                    continue;
                }
            }
            if (entry != null) {
                writeEntry(entry);
                continue;
            }
            long nowNanos = System.nanoTime();
            if (nowNanos - lastFlushNanos >= FLUSH_INTERVAL_MS * 1_000_000L) {
                flushQuietly();
                lastFlushNanos = nowNanos;
            }
            if (stopping || interrupted) {
                // Push the buffers out even if close() has already given up waiting, so an
                // unclean stop still leaves everything written so far on disk.
                flushQuietly();
                return;
            }
        }
    }

    private void writeEntry(Entry entry) {
        if (capped) {
            return;
        }
        if (entry.dest == META) {
            writeMetaFile(entry.data);
            return;
        }
        try {
            OutputStream out = entry.dest == EVENTS ? events : signals;
            out.write(entry.data);
            if (bytesWritten.addAndGet(entry.data.length) > MAX_BYTES) {
                capped = true;
                StringBuilder line = openEvent("run", "truncated");
                line.append("}\n");
                events.write(bytes(line));
                flushQuietly();
            }
        } catch (IOException e) {
            // Storage is gone or full: stop writing rather than throw on every loop.
            capped = true;
            warnOnce("RunLog write failed, logging stopped: " + e);
        }
    }

    private void flushQuietly() {
        try {
            signals.flush();
            events.flush();
        } catch (IOException e) {
            capped = true;
            warnOnce("RunLog flush failed, logging stopped: " + e);
        }
    }

    /**
     * One Driver Station warning per run. Writer thread only. Without this a full or unmounted
     * /sdcard stops logging silently, and the result is indistinguishable from a hard kill.
     */
    private void warnOnce(String message) {
        if (!warned) {
            warned = true;
            RobotLog.addGlobalWarningMessage(message);
        }
    }

    // ------------------------------------------------------------------ internals

    /** Opens an event object and leaves it unterminated for extra fields. */
    private StringBuilder openEvent(String type, String event) {
        StringBuilder line = new StringBuilder(96).append('{').append("\"t\":");
        appendFixed(line, elapsedSeconds(), TIME_DECIMALS);
        line.append(",\"type\":\"").append(type).append("\",\"event\":\"").append(event).append('"');
        return line;
    }

    private void enqueue(int dest, StringBuilder line) {
        offer(new Entry(dest, bytes(line)));
    }

    private void offer(Entry entry) {
        if (!queue.offer(entry)) {
            dropped.incrementAndGet();
        }
    }

    /** Builds meta.json on the calling thread; the bytes are written by the writer. */
    private StringBuilder buildMeta() {
        StringBuilder meta = new StringBuilder(256);
        meta.append("{\"schema\":1,\"opMode\":\"");
        appendEscaped(meta, opModeName);
        meta.append("\",\"startedWallClockMs\":")
                .append(startedWallClockMs)
                .append(",\"columns\":[\"t_s\",\"loop\"");
        for (int i = 0; i < columns.size(); i++) {
            meta.append(",\"");
            appendEscaped(meta, columns.get(i));
            meta.append('"');
        }
        meta.append("]}\n");
        return meta;
    }

    /**
     * Writer thread only: opens, writes and fsyncs meta.json. This is why the meta bytes travel
     * through the queue instead of being written where they are built, which would put an fsync on
     * the control loop in the first loop after Play.
     */
    private void writeMetaFile(byte[] data) {
        FileOutputStream out = null;
        try {
            out = new FileOutputStream(new File(dir, "meta.json"));
            out.write(data);
            out.getFD().sync();
        } catch (IOException e) {
            warnOnce("RunLog meta.json failed: " + e);
        } finally {
            closeQuietly(out);
        }
    }

    private double elapsedSeconds() {
        return (System.nanoTime() - startNanos) / 1e9;
    }

    private long elapsedMs() {
        return (System.nanoTime() - startNanos) / 1_000_000L;
    }

    private void syncAndClose(OutputStream buffered, FileOutputStream file) {
        try {
            buffered.flush();
            file.getFD().sync();
        } catch (IOException ignored) {
            // Nothing useful to do while stopping.
        }
        closeQuietly(buffered);
    }

    private static RunLog degrade(String reason) {
        RobotLog.addGlobalWarningMessage("RunLog disabled: " + reason);
        return disabled();
    }

    private static void closeQuietly(java.io.Closeable stream) {
        if (stream == null) {
            return;
        }
        try {
            stream.close();
        } catch (IOException ignored) {
            // Closing failures cannot be recovered from here.
        }
    }

    private static byte[] bytes(StringBuilder line) {
        return line.toString().getBytes(UTF_8);
    }

    /**
     * Keeps the newest {@link #MAX_RUNS} run directories, deleting older ones with contents.
     * {@code current} is never deleted: the hub clock can read earlier than existing runs, which
     * would otherwise sort the just-created directory first and unlink the run being written.
     */
    private static void prune(File parentDir, File current) {
        File[] children = parentDir.listFiles(File::isDirectory);
        if (children == null || children.length <= MAX_RUNS) {
            return;
        }
        Arrays.sort(children, (a, b) -> a.getName().compareTo(b.getName()));
        for (int i = 0, over = children.length - MAX_RUNS; i < children.length && over > 0; i++) {
            if (children[i].equals(current)) {
                continue;
            }
            deleteRecursively(children[i]);
            over--;
        }
    }

    private static void deleteRecursively(File file) {
        File[] children = file.listFiles();
        if (children != null) {
            for (int i = 0; i < children.length; i++) {
                deleteRecursively(children[i]);
            }
        }
        file.delete();
    }

    /** Directory-safe OpMode name. */
    private static String sanitize(String name) {
        String safe = name.replaceAll("[^A-Za-z0-9._-]", "_");
        return safe.isEmpty() ? "OpMode" : safe;
    }

    /**
     * Appends {@code value} with exactly {@code decimals} digits after the point, allocating
     * nothing. Appends nothing, leaving the CSV field empty, for NaN, infinity, and any magnitude
     * whose scaled value would not fit a long.
     */
    private static void appendFixed(StringBuilder out, double value, int decimals) {
        long scale = 1;
        for (int i = 0; i < decimals; i++) {
            scale *= 10;
        }
        // Negated so NaN, which fails every comparison, is rejected here too.
        if (!(Math.abs(value) < (double) Long.MAX_VALUE / scale)) {
            return;
        }
        long scaled = Math.round(value * scale);
        if (scaled < 0) {
            out.append('-');
            scaled = -scaled;
        }
        out.append(scaled / scale).append('.');
        long fraction = scaled % scale;
        for (long divisor = scale / 10; divisor >= 1; divisor /= 10) {
            out.append((char) ('0' + (fraction / divisor) % 10));
        }
    }

    /** Escapes a JSON string body: quote, backslash, and controls below 0x20. */
    private static void appendEscaped(StringBuilder out, String value) {
        if (value == null) {
            return;
        }
        for (int i = 0; i < value.length(); i++) {
            char c = value.charAt(i);
            switch (c) {
                case '"':
                    out.append("\\\"");
                    break;
                case '\\':
                    out.append("\\\\");
                    break;
                case '\n':
                    out.append("\\n");
                    break;
                case '\r':
                    out.append("\\r");
                    break;
                case '\t':
                    out.append("\\t");
                    break;
                default:
                    if (c < 0x20) {
                        out.append(String.format(Locale.US, "\\u%04x", (int) c));
                    } else {
                        out.append(c);
                    }
            }
        }
    }
}
