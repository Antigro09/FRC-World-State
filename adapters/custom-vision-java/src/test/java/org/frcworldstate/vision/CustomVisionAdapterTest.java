package org.frcworldstate.vision;

import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalLong;
import java.util.function.Consumer;
import org.customvision.protocol.ClockMapper;
import org.customvision.protocol.Packet;
import org.customvision.protocol.ProtocolDecoder;
import org.customvision.protocol.SourceKey;
import org.customvision.protocol.SourceSession;
import org.customvision.protocol.SyncSnapshot;
import org.customvision.protocol.TimeVersion;
import org.customvision.protocol.TransportSample;
import org.frcworldstate.core.Geometry;
import org.frcworldstate.core.PoseHistory;
import org.frcworldstate.core.World;
import org.frcworldstate.core.WorldEngine;
import static org.frcworldstate.vision.CustomVisionAdapter.*;

/** Small CPU-only synthetic tests, plus optional exact producer bytes. Never hardware timing evidence. */
public final class CustomVisionAdapterTest {
    private static int checks;
    private static final String CAL = "sha256:" + "a".repeat(64), MOUNT = "sha256:" + "b".repeat(64);
    private static final SourceKey SOURCE = new SourceKey("/CustomVision/fixture/front_objects", "front_objects", "object");
    private static final long NOW = 1_020_999;
    private static void check(boolean condition, String message) { checks++; if (!condition) throw new AssertionError(message); }
    private static Map<String, Object> map(Object... pairs) {
        Map<String, Object> out = new LinkedHashMap<>();
        for (int i = 0; i < pairs.length; i += 2) out.put((String) pairs[i], pairs[i + 1]);
        return out;
    }
    private static Policy policy(double height) { return new Policy(height, .000002, .5, 10, .04, .25, .000001, 20_000, 100_000, 100_000, 8); }
    private static Binding binding(Packet p, TimeVersion version, long epoch) {
        return new Binding(p.source(), PROFILE, p.bootId(), "robot-boot-1", epoch, 2, version,
                (String) p.fields().get("calibration_revision"), (String) p.fields().get("mount_revision"),
                Optional.empty(), "same-exposure-or-shared-detector", "target-plane-profile-v1");
    }
    private static CustomVisionAdapter adapter(Packet p, TimeVersion version) { return new CustomVisionAdapter(binding(p, version, 0), policy(0)); }
    private static Packet packet(Map<String, Object> fields) {
        return new Packet(SOURCE, (Long) fields.get("schema_version"), (String) fields.get("boot_id"),
                (Long) fields.get("frame_id"), OptionalLong.of((Long) fields.get("packet_seq")), CAL + "|" + MOUNT + "|null",
                (Boolean) fields.get("connected"), (Long) fields.get("capture_server_us"), (Boolean) fields.get("time_sync_valid"), fields);
    }
    private static Packet synthetic() {
        var target = map("valid", true, "frame", "robot_relative_at_capture_wpilib_nwu",
                "translation_m", List.of(1., 0., 0.), "target_height_m", 0.,
                "method", "calibrated_ray_target_height_plane", "approximate", true,
                "observed", true, "predicted", false, "capture_monotonic_us", 1_000_123L,
                "track_id", 1L, "label", "synthetic_piece", "confidence", .9,
                "uncertainty", map("model", "first_order_configured_pixel_height_pitch_mount",
                        "covariance_xy_m2", List.of(List.of(.001, 0.), List.of(0., .002))));
        return packet(map("schema_version", 2L, "protocol_profile", PROFILE, "boot_id", "vision-boot-1",
                "frame_id", 42L, "packet_seq", 8L, "connected", true, "capture_server_us", 3_000_123L,
                "time_sync_valid", true, "capture_monotonic_us", 1_000_123L,
                "calibration_revision", CAL, "mount_revision", MOUNT, "field_layout_revision", null,
                "timing", map("clock_domain", "nt_server", "timestamp_unit", "us", "capture_event", "host_frame_read_complete",
                        "capture_correction_verified", true, "capture_correction_uncertainty_ms", .001),
                "objects", map("valid", true, "frame", "robot_relative_at_capture_wpilib_nwu", "motion_compensated", false,
                        "targets", List.of(target))));
    }
    private static Packet edit(Packet packet, Consumer<Map<String, Object>> change) {
        Map<String, Object> fields = new LinkedHashMap<>(packet.fields()); change.accept(fields); return packet(fields);
    }
    private static Packet target(Packet p, Consumer<Map<String, Object>> change) {
        return edit(p, fields -> {
            var objects = new LinkedHashMap<>(p.family("objects").orElseThrow());
            List<?> original = (List<?>) objects.get("targets");
            @SuppressWarnings("unchecked") var first = new LinkedHashMap<>((Map<String, Object>) original.get(0));
            change.accept(first); List<Object> targets = new ArrayList<>(original); targets.set(0, first);
            objects.put("targets", targets); fields.put("objects", objects);
        });
    }
    private static Packet objects(Packet p, Consumer<Map<String, Object>> change) {
        return edit(p, fields -> { var object = new LinkedHashMap<>(p.family("objects").orElseThrow()); change.accept(object); fields.put("objects", object); });
    }
    private static ClockMapper.Result mapping(Packet p, TimeVersion version) {
        long scale = version == TimeVersion.WPILIB_2026_MICROSECONDS ? 1 : 1000;
        var sample = new TransportSample("synthetic", 1_010_999 * scale, 3_010_999 * scale, NOW * 1000, NOW * 1000, 2);
        var sync = new SyncSnapshot(version, OptionalLong.of(2_000_000 * scale), "robot-boot-1", true,
                999, 1000, NOW * 1000, 2, "synthetic offset check; not hardware evidence");
        return new ClockMapper("robot-boot-1").map(p, sample, sync, NOW * 1000 + 999);
    }
    private static SourceSession.Result admit(Packet p) {
        return new SourceSession.Result(SourceSession.Kind.ACCEPTED, Optional.of(p), SourceSession.Reason.NONE);
    }
    private static void expect(Reason reason, CustomVisionAdapter a, Packet p, ClockMapper.Result m, long now) {
        Result r = a.adapt(p, admit(p), m, now); check(r.reason() == reason, "expected " + reason + " got " + r);
        check(!r.provesFreeSpace(), "adapter cannot establish free space");
        check(r.frame().isPresent() == (reason == Reason.ACCEPTED), "typed measurement presence");
        check(r.provenance().rawPacket() == p && r.provenance().clockMapping() == m, "full immutable raw provenance retained");
    }
    private static World.EgoState ego(long time, double x, double y, double heading) {
        return new World.EgoState(time, new Geometry.Pose2(new Geometry.Vec2(x, y), heading), Geometry.Velocity2.zero(),
                new Geometry.Uncertainty(.01, 0, .01), true);
    }
    private static void coreIntegration(Packet p, ClockMapper.Result mapped) {
        CustomVisionAdapter a = adapter(p, TimeVersion.WPILIB_2026_MICROSECONDS);
        World.ObservationFrame frame = a.adapt(p, admit(p), mapped, NOW).frame().orElseThrow();
        var history = new PoseHistory(4, 1_000_000);
        var engine = new WorldEngine(new WorldEngine.Config(8, 8, 4, 100_000, 250_000, 10_000, 10_000,
                1, .5, 1, .01, 5, 1, .02, 10, .02, .1, .2, .1, false), history,
                new Geometry.FieldIdentity("OFFSEASON_2026", "synthetic", "map-v1"));
        engine.addEgo(ego(frame.stamp().captureUs(), 1, 2, Math.PI / 2));
        engine.addEgo(ego(NOW, 1.05, 2.01, Math.PI / 2));
        check(engine.accept(frame, NOW).acceptedMeasurements() == 1, "one raw target becomes one core measurement");
        var snapshot = engine.publish(ego(NOW, 1.05, 2.01, Math.PI / 2));
        var track = snapshot.tracks().get(0);
        check(Math.abs(track.positionM().x() - 1) < 1e-10 && Math.abs(track.positionM().y() - 3) < 1e-10,
                "capture pose projects target; publication/current pose cannot replace it");
        check(track.lastMeasurementUs() == 1_000_123, "measurement time remains capture time");
        Packet republished = edit(p, f -> f.put("packet_seq", 9L));
        var repeat = a.adapt(republished, admit(republished), mapping(republished, TimeVersion.WPILIB_2026_MICROSECONDS), NOW).frame().orElseThrow();
        check(engine.accept(repeat, NOW).status() == WorldEngine.FrameStatus.DUPLICATE,
                "packet sequence does not turn duplicate capture into an independent measurement");
        Packet dead = edit(p, f -> f.put("connected", false));
        check(a.adapt(dead, admit(dead), mapped, NOW).frame().isEmpty(), "dead source cannot insert empty-space observation");
        check(engine.publish(ego(NOW + 1000, 1.05, 2.01, Math.PI / 2)).tracks().size() == 1, "unavailable camera cannot delete occupancy");
    }

    private static void lifecycle(Packet p, ClockMapper.Result mapped) {
        var session = new SourceSession(SOURCE);
        var a = adapter(p, TimeVersion.WPILIB_2026_MICROSECONDS);
        Packet first = edit(p, f -> { f.put("frame_id", 41L); f.put("packet_seq", 7L); });
        var firstTransport = new TransportSample("synthetic retained first publication", 1_010_998, 3_010_998,
                NOW * 1000, NOW * 1000, 2);
        var pending = session.accept(first, firstTransport, NOW * 1000);
        check(pending.kind() == SourceSession.Kind.PENDING, "first retained publication needs advancing adoption");
        check(a.adapt(p, pending, mapped, NOW).reason() == Reason.NO_NEW_MEASUREMENT,
                "unadopted source cannot emit normalized observations");
        var currentTransport = new TransportSample("synthetic fresh advancing publication", 1_010_999, 3_010_999,
                NOW * 1000, NOW * 1000, 2);
        var admitted = session.accept(p, currentTransport, NOW * 1000);
        check(admitted.newlyAcceptedMeasurement().orElseThrow().equals(p), "actual lifecycle emits fresh coherent packet");
        check(a.adapt(p, admitted, mapped, NOW).status() == Status.ACCEPTED, "actual lifecycle admission normalizes one capture");
        Packet different = edit(p, f -> f.put("packet_seq", 10L));
        check(a.adapt(different, admitted, mapped, NOW).reason() == Reason.LIFECYCLE_MISMATCH,
                "another coherent packet cannot borrow a lifecycle admission");
        var duplicate = session.accept(p, currentTransport, NOW * 1000);
        check(a.adapt(p, duplicate, mapped, NOW).reason() == Reason.NO_NEW_MEASUREMENT,
                "duplicate status is not a newly accepted measurement");
        Packet watchdog = edit(p, f -> { f.put("connected", false); f.put("packet_seq", 9L); });
        var watchdogTransport = new TransportSample("synthetic same-frame watchdog", 1_011_000, 3_011_000,
                NOW * 1000, NOW * 1000, 2);
        var invalidation = session.accept(watchdog, watchdogTransport, NOW * 1000);
        check(invalidation.reason() == SourceSession.Reason.INVALIDATED_FRAME && !session.status().actionable(),
                "same-frame advancing invalidation clears action before capture dedup");
        check(a.adapt(watchdog, invalidation, mapped, NOW).reason() == Reason.DISCONNECTED,
                "accepted watchdog remains unavailable in adapter");
        var afterWatchdogTransport = new TransportSample("synthetic same-frame revival attempt", 1_011_001, 3_011_001,
                NOW * 1000, NOW * 1000, 2);
        var tombstoned = session.accept(different, afterWatchdogTransport, NOW * 1000);
        check(a.adapt(different, tombstoned, mapped, NOW).reason() == Reason.NO_NEW_MEASUREMENT,
                "same-frame validity cannot revive a tombstoned observation");
    }

    public static void main(String[] args) throws Exception {
        Packet p = synthetic(); var version = TimeVersion.WPILIB_2026_MICROSECONDS;
        ClockMapper.Result mapped = mapping(p, version); check(mapped.accepted(), "synthetic explicitly verified mapping");
        CustomVisionAdapter a = adapter(p, version); expect(Reason.ACCEPTED, a, p, mapped, NOW);
        var frame = a.adapt(p, admit(p), mapped, NOW).frame().orElseThrow();
        check(frame.stamp().captureUs() == 1_000_123 && frame.stamp().publicationUs() == 1_010_999,
                "mapped nanosecond conversion floors backwards, retaining separate capture/publication instants");
        check(frame.stamp().sequence() == 42 && p.packetSeq().orElseThrow() == 8, "capture identity distinct from publication sequence");
        check(frame.observations().get(0).relativeUncertainty().equals(new Geometry.Uncertainty(.04, 0, .04)),
                "explicit covariance floor; confidence is not uncertainty");
        check(frame.stamp().sourceId().contains(p.source().namespace()) && frame.stamp().sourceId().contains(PROFILE),
                "full source/profile identity");
        check(a.adapt(p, admit(p), mapped, NOW).provenance().observedFieldLayoutRevision().isEmpty(),
                "field layout is optional for capture-relative object geometry");

        var nsVersion = TimeVersion.WPILIB_2027_ALPHA7_NANOSECONDS;
        var nsMapped = mapping(p, nsVersion); check(nsMapped.accepted(), "synthetic alpha7 ns mapping");
        expect(Reason.ACCEPTED, adapter(p, nsVersion), p, nsMapped, NOW);
        check(adapter(p, nsVersion).adapt(p, admit(p), nsMapped, NOW).frame().orElseThrow().equals(frame), "2026 and alpha7 normalize to identical us geometry");
        expect(Reason.CLOCK_MISMATCH, a, p, nsMapped, NOW);
        expect(Reason.STALE, a, p, mapped, NOW + 100_001);
        expect(Reason.FUTURE, a, p, mapped, 1_000_000);
        expect(Reason.DISCONNECTED, a, edit(p, f -> f.put("connected", false)), mapped, NOW);
        expect(Reason.MISSING_FAMILY, a, edit(p, f -> f.remove("objects")), mapped, NOW);
        expect(Reason.INVALID_FAMILY, a, objects(p, f -> f.put("valid", false)), mapped, NOW);
        expect(Reason.EMPTY_OBSERVATIONS, a, objects(p, f -> f.put("targets", List.of())), mapped, NOW);
        expect(Reason.REVISION_MISMATCH, a, edit(p, f -> f.put("mount_revision", null)), mapped, NOW);
        expect(Reason.REVISION_MISMATCH, a, edit(p, f -> f.put("calibration_revision", "sha256:" + "c".repeat(64))), mapped, NOW);
        expect(Reason.BOOT_MISMATCH, a, edit(p, f -> f.put("boot_id", "retained-old-boot")), mapped, NOW);
        expect(Reason.PROFILE_MISMATCH, a, edit(p, f -> f.put("protocol_profile", "legacy-schema2")), mapped, NOW);
        Packet wrongSource = new Packet(new SourceKey("/another/source", "front_objects", "object"), p.schemaVersion(), p.bootId(),
                p.frameId(), p.packetSeq(), p.revision(), p.connected(), p.captureServerUs(), p.timeSyncValid(), p.fields());
        expect(Reason.SOURCE_MISMATCH, a, wrongSource, mapped, NOW);
        Packet wrongCapture = edit(p, f -> f.put("capture_server_us", 3_000_124L));
        expect(Reason.CLOCK_MISMATCH, a, wrongCapture, mapped, NOW);
        Packet unverified = edit(p, f -> {
            var timing = new LinkedHashMap<>(p.family("timing").orElseThrow()); timing.put("capture_correction_verified", false); f.put("timing", timing);
        });
        expect(Reason.CLOCK_REJECTED, a, unverified, mapping(unverified, version), NOW);
        expect(Reason.CLOCK_MISMATCH, a, unverified, mapped, NOW); // an accepted mapping from another raw timing block cannot override it
        expect(Reason.UNSUPPORTED_ESTIMATE, a, objects(p, f -> f.put("motion_compensated", true)), mapped, NOW);
        expect(Reason.UNSUPPORTED_ESTIMATE, a, target(p, f -> f.put("predicted", true)), mapped, NOW);
        expect(Reason.UNSUPPORTED_ESTIMATE, a, target(p, f -> f.put("observed", false)), mapped, NOW);
        expect(Reason.INVALID_GEOMETRY, a, target(p, f -> f.put("frame", "opencv_right_down_forward")), mapped, NOW);
        expect(Reason.INVALID_GEOMETRY, a, target(p, f -> f.remove("translation_m")), mapped, NOW);
        expect(Reason.INVALID_GEOMETRY, a, target(p, f -> f.put("translation_m", List.of(1., Double.NaN, 0.))), mapped, NOW);
        expect(Reason.INVALID_GEOMETRY, a, target(p, f -> f.put("translation_m", List.of(11., 0., 0.))), mapped, NOW);
        expect(Reason.INVALID_GEOMETRY, a, target(p, f -> f.put("capture_monotonic_us", 99L)), mapped, NOW);
        expect(Reason.TARGET_PLANE_MISMATCH, a, target(p, f -> f.put("target_height_m", .5)), mapped, NOW);
        expect(Reason.INVALID_QUALITY, a, target(p, f -> f.remove("confidence")), mapped, NOW);
        expect(Reason.INVALID_QUALITY, a, target(p, f -> f.put("confidence", 1.2)), mapped, NOW);
        expect(Reason.INVALID_QUALITY, a, target(p, f -> f.put("confidence", .4)), mapped, NOW);
        expect(Reason.INVALID_UNCERTAINTY, a, target(p, f -> f.remove("uncertainty")), mapped, NOW);
        expect(Reason.INVALID_UNCERTAINTY, a, target(p, f -> f.put("uncertainty", map("model", "pixels-as-variance", "covariance_xy_m2", List.of(List.of(.1, 0.), List.of(0., .1))))), mapped, NOW);
        expect(Reason.INVALID_UNCERTAINTY, a, target(p, f -> f.put("uncertainty", map("model", "first_order_configured_pixel_height_pitch_mount", "covariance_xy_m2", List.of(List.of(.1, .1), List.of(0., .1))))), mapped, NOW);
        expect(Reason.INVALID_UNCERTAINTY, a, target(p, f -> f.put("uncertainty", map("model", "first_order_configured_pixel_height_pitch_mount", "covariance_xy_m2", List.of(List.of(.1, .2), List.of(.2, .1))))), mapped, NOW);
        expect(Reason.INVALID_UNCERTAINTY, a, target(p, f -> f.put("uncertainty", map("model", "first_order_configured_pixel_height_pitch_mount", "covariance_xy_m2", List.of(List.of(.3, 0.), List.of(0., .1))))), mapped, NOW);
        Packet duplicate = objects(p, f -> { List<?> targets = (List<?>) f.get("targets"); f.put("targets", List.of(targets.get(0), targets.get(0))); });
        expect(Reason.INVALID_GEOMETRY, a, duplicate, mapped, NOW);
        Packet crowded = objects(p, f -> { Object t = ((List<?>) f.get("targets")).get(0); f.put("targets", java.util.Collections.nCopies(9, t)); });
        expect(Reason.CAPACITY, a, crowded, mapped, NOW);
        Packet belowPlane = target(p, f -> { f.put("target_height_m", -.2); f.put("translation_m", List.of(1., 0., -.2)); });
        expect(Reason.ACCEPTED, new CustomVisionAdapter(binding(p, version, 0), policy(-.2)), belowPlane, mapped, NOW);
        coreIntegration(p, mapped);
        lifecycle(p, mapped);
        if (args.length == 1) producerFixtures(Path.of(args[0]));
        else if (args.length != 0) throw new IllegalArgumentException("optional argument: producer fixture directory");
        else System.out.println("Producer golden-byte match NOT RUN; supply the pinned producer fixture directory");
        System.out.println("CustomVisionAdapterTest passed " + checks + " assertions");
    }

    private static void producerFixtures(Path dir) throws Exception {
        Path file = dir.resolve("objects_compact_covariance_selection.json"); byte[] bytes = Files.readAllBytes(file);
        String hash = HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(bytes));
        check(hash.equals("c0e5c0ea9241423e1d3aa8fe19373931cf3fab080ced9af2b9021d9eaf6fc6d9"), "exact producer coherent-string pin");
        Packet raw = new ProtocolDecoder().decode(SOURCE, new String(bytes, java.nio.charset.StandardCharsets.UTF_8));
        check(raw.familyValid("objects") && ((List<?>) raw.family("objects").orElseThrow().get("targets")).size() == 2,
                "actual producer bytes decode canonical targets and compact references");
        // The actual producer fixture explicitly has unverified capture correction. It must abstain.
        var sync = new SyncSnapshot(TimeVersion.WPILIB_2026_MICROSECONDS, OptionalLong.of(2_000_000L), "robot-boot-1", true,
                0, 1000, 1_234_567_920_123_000L, 2, "synthetic test only");
        var sample = new TransportSample(new String(bytes, java.nio.charset.StandardCharsets.UTF_8), 1_234_567_900_123L,
                1_234_569_900_123L, 1_234_567_920_123_000L, 1_234_567_920_123_000L, 2);
        var rejected = new ClockMapper("robot-boot-1").map(raw, sample, sync, 1_234_567_920_123_000L);
        check(rejected.rejection().orElseThrow().reason() == ClockMapper.Reason.CAPTURE_CORRECTION_UNVERIFIED, "producer fixture makes no timing claim");
        var a = new CustomVisionAdapter(binding(raw, TimeVersion.WPILIB_2026_MICROSECONDS, 0), policy(0));
        expect(Reason.CLOCK_REJECTED, a, raw, rejected, 1_234_567_920_123L);

        // Named synthetic timing modification: geometry bytes are decoded unchanged, but this is not a producer/hardware timing qualification.
        var fields = new LinkedHashMap<>(raw.fields());
        var timing = new LinkedHashMap<>(raw.family("timing").orElseThrow());
        timing.put("capture_correction_verified", true); timing.put("capture_correction_uncertainty_ms", .001); fields.put("timing", timing);
        Packet syntheticTiming = new Packet(raw.source(), raw.schemaVersion(), raw.bootId(), raw.frameId(), raw.packetSeq(),
                raw.revision(), raw.connected(), raw.captureServerUs(), raw.timeSyncValid(), fields);
        var accepted = new ClockMapper("robot-boot-1").map(syntheticTiming, sample, sync, 1_234_567_920_123_000L);
        check(accepted.accepted(), "synthetic fixture timing map");
        var result = a.adapt(syntheticTiming, admit(syntheticTiming), accepted, 1_234_567_920_123L);
        check(result.status() == Status.ACCEPTED && result.frame().orElseThrow().observations().size() == 2,
                "producer canonical geometry normalizes under explicitly synthetic timing");
        check(result.frame().orElseThrow().observations().get(1).robotRelativeM().y() > 0,
                "producer mounted NWU +Y left retained without optical-axis sign conversion");
        System.out.println("Producer geometry fixture SHA-256 matched; hardware capture correction remains unqualified");
    }
}
