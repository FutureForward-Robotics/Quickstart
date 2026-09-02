package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

/** RunLog against a temporary directory, so nothing touches AppUtil or the hub filesystem. */
class RunLogTest {

    private static final Charset UTF_8 = Charset.forName("UTF-8");

    @TempDir File logDir;

    private List<String> lines(RunLog log, String file) throws IOException {
        return Files.readAllLines(new File(log.directory(), file).toPath(), UTF_8);
    }

    @Test
    void writesHeaderThenOneRowPerLoop() throws IOException {
        RunLog log = RunLog.open(logDir, "TeleOp Red");
        log.addSignal("a", () -> 1.5);
        log.addSignal("b", () -> -0.25);
        log.addFlag("flag", () -> true);

        log.writeLoop(0);
        log.writeLoop(1);
        log.writeLoop(2);
        log.close();

        List<String> csv = lines(log, "signals.csv");
        assertEquals(4, csv.size(), csv.toString());
        assertEquals("t_s,loop,a,b,flag", csv.get(0));
        for (int i = 1; i < csv.size(); i++) {
            String[] fields = csv.get(i).split(",", -1);
            assertEquals(5, fields.length, csv.get(i));
            assertEquals(String.valueOf(i - 1), fields[1]);
            assertEquals("1.5000", fields[2]);
            assertEquals("-0.2500", fields[3]);
            assertEquals("1", fields[4], "booleans are 1/0, not true/false");
        }
    }

    @Test
    void metaJsonListsTheColumns() throws IOException {
        RunLog log = RunLog.open(logDir, "TeleOp Red");
        log.addSignal("drive.x", () -> 0);
        log.addFlag("drive.busy", () -> false);
        log.writeLoop(0);
        log.close();

        String meta = String.join("", lines(log, "meta.json"));
        assertTrue(meta.contains("\"opMode\":\"TeleOp Red\""), meta);
        assertTrue(meta.contains("\"t_s\""), meta);
        assertTrue(meta.contains("\"loop\""), meta);
        assertTrue(meta.contains("\"drive.x\""), meta);
        assertTrue(meta.contains("\"drive.busy\""), meta);
    }

    @Test
    void eventsAreOnePerLine() throws IOException {
        RunLog log = RunLog.open(logDir, "Auto");
        log.event("note", "driver \"took\" the c:\\drivetrain");
        log.commandEvent("start", "ScoreHigh", 1234567);
        log.commandEvent("interrupt", "ScoreHigh", 1234567);
        log.close();

        // run/start, the note, two command events, run/end.
        List<String> events = lines(log, "events.jsonl");
        assertEquals(5, events.size(), events.toString());
        for (int i = 0; i < events.size(); i++) {
            String line = events.get(i);
            assertTrue(line.startsWith("{") && line.endsWith("}"), line);
            assertTrue(line.contains("\"t\":"), line);
        }
        assertTrue(events.get(0).contains("\"event\":\"start\""), events.get(0));
        assertTrue(events.get(1).contains("\\\"took\\\""), events.get(1));
        assertTrue(events.get(1).contains("c:\\\\drivetrain"), events.get(1));
        assertTrue(events.get(4).contains("\"event\":\"end\""), events.get(4));
    }

    @Test
    void registeringASignalAfterTheFirstRowThrows() {
        RunLog log = RunLog.open(logDir, "Auto");
        log.addSignal("a", () -> 0);
        log.writeLoop(0);

        assertThrows(IllegalStateException.class, () -> log.addSignal("late", () -> 1));
        assertThrows(IllegalStateException.class, () -> log.addFlag("late", () -> true));
        log.close();
    }

    @Test
    void aThrowingSupplierLeavesAnEmptyFieldAndKeepsGoing() throws IOException {
        RunLog log = RunLog.open(logDir, "Auto");
        log.addSignal("a", () -> 1);
        log.addSignal("bad", () -> {
            throw new IllegalStateException("sensor unplugged");
        });
        log.addSignal("c", () -> 2);

        log.writeLoop(0);
        log.writeLoop(1);
        log.close();

        List<String> csv = lines(log, "signals.csv");
        assertEquals(3, csv.size(), csv.toString());
        for (int i = 1; i < csv.size(); i++) {
            String[] fields = csv.get(i).split(",", -1);
            assertEquals(5, fields.length, csv.get(i));
            assertEquals("1.0000", fields[2]);
            assertEquals("", fields[3], "a throwing supplier writes an empty field");
            assertEquals("2.0000", fields[4]);
        }
    }

    @Test
    void slowSignalRepeatsTheCachedValueBetweenSamples() throws IOException {
        AtomicInteger samples = new AtomicInteger();
        RunLog log = RunLog.open(logDir, "Auto");
        log.addSlowSignal("battery", () -> samples.incrementAndGet(), 60_000);

        log.writeLoop(0);
        log.writeLoop(1);
        log.writeLoop(2);
        log.close();

        assertEquals(1, samples.get(), "an expensive source is sampled once per interval");
        List<String> csv = lines(log, "signals.csv");
        assertEquals("1.0000", csv.get(1).split(",", -1)[2]);
        assertEquals("1.0000", csv.get(3).split(",", -1)[2]);
    }

    @Test
    void disabledSinkWritesNothingAndReportsDisabled() {
        RunLog log = RunLog.disabled();
        log.addSignal("a", () -> 0);
        log.addFlag("b", () -> true);
        log.addSlowSignal("c", () -> 0, 100);
        log.writeLoop(0);
        log.event("note", "ignored");
        log.commandEvent("start", "Nothing", 1);
        log.close();

        assertFalse(log.isEnabled());
        assertEquals(0, log.dropped());
        assertEquals(null, log.directory());
        assertEquals(0, logDir.listFiles().length, "no files are created");
    }

    @Test
    void closeIsIdempotent() throws IOException {
        RunLog log = RunLog.open(logDir, "Auto");
        log.addSignal("a", () -> 1);
        log.writeLoop(0);
        log.close();
        List<String> afterFirstClose = lines(log, "events.jsonl");

        log.close();

        assertEquals(afterFirstClose, lines(log, "events.jsonl"));
        assertEquals(2, lines(log, "signals.csv").size());
    }

    @Test
    void noDropsUnderNormalUse() {
        RunLog log = RunLog.open(logDir, "Auto");
        log.addSignal("a", () -> 1);
        log.addFlag("b", () -> false);
        for (int i = 0; i < 500; i++) {
            log.writeLoop(i);
        }
        log.close();

        assertEquals(0, log.dropped());
    }

    @Test
    void duplicateSignalNamesAreRejected() {
        RunLog log = RunLog.open(logDir, "Auto");
        log.addSignal("lift.pos", () -> 1);

        assertThrows(IllegalArgumentException.class, () -> log.addSignal("lift.pos", () -> 2));
        assertThrows(IllegalArgumentException.class, () -> log.addFlag("lift.pos", () -> true));
        log.close();
    }

    @Test
    void magnitudesTooLargeToScaleLeaveTheFieldEmpty() throws IOException {
        RunLog log = RunLog.open(logDir, "Auto");
        log.addSignal("big", () -> 1e15);
        log.addSignal("negBig", () -> -1e15);
        log.addSignal("ok", () -> 1.5);
        log.writeLoop(0);
        log.close();

        String[] fields = lines(log, "signals.csv").get(1).split(",", -1);
        assertEquals("", fields[2], "1e15 overflows the scaled long, so no digits are written");
        assertEquals("", fields[3]);
        assertEquals("1.5000", fields[4], "a normal value in the same row is unaffected");
    }

    @Test
    void pruneKeepsTheRunItJustCreated() throws IOException {
        // Names sort by timestamp, so a hub clock reading earlier than every existing run puts the
        // new directory first. It must survive anyway.
        for (int i = 0; i < 30; i++) {
            File old = new File(logDir, String.format("29990101-%06d-Old", i));
            assertTrue(old.mkdirs());
        }

        RunLog log = RunLog.open(logDir, "Auto");
        log.addSignal("a", () -> 1);
        log.writeLoop(0);
        log.close();

        assertTrue(log.isEnabled(), "the run opened");
        assertTrue(log.directory().isDirectory(), "the new run directory was not pruned");
        assertEquals(2, lines(log, "signals.csv").size(), "and it is still readable");
    }
}
