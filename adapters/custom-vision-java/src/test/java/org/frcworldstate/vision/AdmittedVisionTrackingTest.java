package org.frcworldstate.vision;

import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalLong;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;
import org.customvision.protocol.ClockMapper;
import org.customvision.protocol.Packet;
import org.customvision.protocol.ProtocolDecoder;
import org.customvision.protocol.RawQueue;
import org.customvision.protocol.SourceKey;
import org.customvision.protocol.SourceSession;
import org.customvision.protocol.SyncSnapshot;
import org.customvision.protocol.TimeVersion;
import org.customvision.protocol.TransportSample;
import org.customvision.protocol.VisionClient;
import org.frcworldstate.core.Geometry;
import org.frcworldstate.core.PoseHistory;
import org.frcworldstate.core.PlannerBackend;
import org.frcworldstate.core.SafetySupervisor;
import org.frcworldstate.core.TargetSelector;
import org.frcworldstate.core.World;
import org.frcworldstate.core.WorldEngine;

/** Real decoder/session/clock/client pipeline over recording fake transport. CPU synthetic, no hardware. */
public final class AdmittedVisionTrackingTest {
    private AdmittedVisionTrackingTest() {}
    private static int checks;
    private static final long BASE_US = 1_234_567_890_123L;
    private static final long SERVER_BASE_US = BASE_US + 2_000_000;
    private static final String ROBOT_BOOT = "synthetic-robot-boot";
    private static final SourceKey SOURCE = new SourceKey("/CustomVision/fixture/front_objects", "front_objects", "object");
    private static final Geometry.FieldIdentity FIELD = new Geometry.FieldIdentity("OFFSEASON_2026", "synthetic-map", "v1");
    private static String golden;

    private static String wire(long frame, long sequence, long captureDeltaUs, String boot,
            boolean connected, boolean syntheticVerifiedTiming) {
        String result = golden.replace("\"frame_id\":42", "\"frame_id\":" + frame)
                .replace("\"packet_seq\":0", "\"packet_seq\":" + sequence)
                .replace("\"boot_id\":\"fixture-boot-a\"", "\"boot_id\":\"" + boot + "\"")
                .replace("\"capture_monotonic_us\":" + BASE_US, "\"capture_monotonic_us\":" + (BASE_US + captureDeltaUs))
                .replace("\"capture_server_us\":" + SERVER_BASE_US, "\"capture_server_us\":" + (SERVER_BASE_US + captureDeltaUs));
        if (!connected) result = result.replace("\"connected\":true", "\"connected\":false");
        // This named modification is simulated timing evidence, not qualification of producer or camera correction.
        if (syntheticVerifiedTiming) result = result.replace("\"capture_correction_verified\":false", "\"capture_correction_verified\":true")
                .replace("\"capture_correction_uncertainty_ms\":null", "\"capture_correction_uncertainty_ms\":0.001");
        return result;
    }
    private static final class FakeQueue implements RawQueue {
        final ArrayDeque<TransportSample> pending = new ArrayDeque<>();
        final TimeVersion version;
        boolean closed;
        FakeQueue(TimeVersion version) { this.version = version; }
        void add(String json, long publicationDeltaUs, long connectionEpoch) {
            long localUs = BASE_US + publicationDeltaUs;
            long scale = version == TimeVersion.WPILIB_2026_MICROSECONDS ? 1 : 1000;
            pending.add(new TransportSample(json, localUs * scale, (localUs + 2_000_000) * scale,
                    localUs * 1000, localUs * 1000, connectionEpoch));
        }
        @Override public Batch read(int maximumPackets) {
            List<TransportSample> samples = new ArrayList<>();
            while (samples.size() < maximumPackets && !pending.isEmpty()) samples.add(pending.removeFirst());
            return new Batch(samples, false);
        }
        @Override public void close() { closed = true; }
    }
    private static final class Pipeline implements AutoCloseable {
        final AtomicLong nowNs = new AtomicLong(BASE_US * 1000);
        final AtomicLong connection = new AtomicLong(2);
        final FakeQueue queue;
        final VisionClient client;
        Pipeline(TimeVersion version) {
            queue = new FakeQueue(version);
            long scale = version == TimeVersion.WPILIB_2026_MICROSECONDS ? 1 : 1000;
            client = new VisionClient(List.of(new VisionClient.Input(SOURCE, queue, () ->
                    new SyncSnapshot(version, OptionalLong.of(2_000_000 * scale), ROBOT_BOOT,
                            true, 0, 1000, nowNs.get(), connection.get(), "synthetic verified matched epochs"))),
                    new ProtocolDecoder(), new ClockMapper(ROBOT_BOOT),
                    new VisionClient.Limits(1, 16, 16, 16, 16, 0), nowNs::get);
        }
        void publish(long frame, long sequence, long captureDeltaUs, long publicationDeltaUs,
                String boot, boolean connected, boolean timingVerified) {
            queue.add(wire(frame, sequence, captureDeltaUs, boot, connected, timingVerified), publicationDeltaUs, connection.get());
            nowNs.set((BASE_US + publicationDeltaUs) * 1000);
            client.poll();
        }
        List<VisionClient.Measurement> drain() { return client.drainMeasurements(nowNs.get()); }
        @Override public void close() { client.close(); }
    }
    private static World.EgoState ego(long deltaUs, double x, double y, double heading) {
        return new World.EgoState(BASE_US + deltaUs, new Geometry.Pose2(new Geometry.Vec2(x, y), heading),
                Geometry.Velocity2.zero(), new Geometry.Uncertainty(.01, 0, .01), true);
    }
    private static WorldEngine engine() {
        return new WorldEngine(new WorldEngine.Config(8, 32, 8, 100_000, 250_000, 10_000, 10_000,
                1, .5, 1, .01, 5, 1, .02, 10, .02, .1, .2, .1, false),
                new PoseHistory(16, 1_000_000), FIELD);
    }
    private static CustomVisionAdapter adapter(Packet packet, TimeVersion version, long worldEpoch, long connectionEpoch) {
        return new CustomVisionAdapter(new CustomVisionAdapter.Binding(SOURCE, CustomVisionAdapter.PROFILE,
                packet.bootId(), ROBOT_BOOT, worldEpoch, connectionEpoch, version,
                (String) packet.fields().get("calibration_revision"), (String) packet.fields().get("mount_revision"),
                Optional.empty(), "configured-common-exposure", "synthetic-floor-profile-v1"),
                new CustomVisionAdapter.Policy(0, .000002, .5, 10, .04, .25, .000001,
                        20_000, 100_000, 100_000, 8));
    }
    private static VisionClient.Measurement adopt(Pipeline pipeline) {
        pipeline.publish(42, 0, 0, 10_000, "fixture-boot-a", true, true);
        check(pipeline.drain().isEmpty(), "initial retained packet is pending adoption");
        check(pipeline.client.drainReceipts().isEmpty(), "pending adoption cannot borrow a receipt or clock result");
        pipeline.publish(43, 1, 20_000, 31_000, "fixture-boot-a", true, true);
        List<VisionClient.Measurement> measurements = pipeline.drain();
        check(measurements.size() == 1, "advancing publication emits exactly one actual admitted measurement");
        return measurements.get(0);
    }
    private static void consumeIntoTracking(TimeVersion version) {
        FakeQueue closedQueue;
        try (Pipeline pipeline = new Pipeline(version)) {
            closedQueue = pipeline.queue;
            VisionClient.Measurement measurement = adopt(pipeline);
            check(measurement.admission().kind() == SourceSession.Kind.ACCEPTED
                    && measurement.admission().newlyAcceptedMeasurement().orElseThrow() == measurement.packet(),
                    "original accepted lifecycle result retains exact packet identity");
            check(measurement.clock().capture().orElseThrow() == measurement.capture(), "full clock outcome retains original mapped capture");
            check(measurement.capture().rawNtTimestamp() == measurement.transport().ntTimestamp()
                    && measurement.capture().connectionEpoch() == measurement.transport().connectionEpoch(), "raw transport provenance retained");
            WorldEngine engine = engine();
            engine.addEgo(ego(20_000, 1, 2, Math.PI / 2));
            engine.addEgo(ego(31_000, 1.05, 2.01, Math.PI / 2));
            var bridge = new VisionTrackBridge(adapter(measurement.packet(), version, 0, 2), engine, pipeline.client);
            var fencedBridge = new VisionTrackBridge(adapter(measurement.packet(), version, 0, 2), engine,
                    (VisionClient.DeliveryGate) (observation, nowNs) -> false);
            var fenced = fencedBridge.acceptRobotNanoseconds(measurement, (BASE_US + 31_000) * 1000);
            check(fenced.normalization().reason() == CustomVisionAdapter.Reason.NO_NEW_MEASUREMENT
                    && fenced.insertedObservations() == 0 && fenced.tracking().isEmpty(),
                    "injected delivery-gate rejection blocks an old admitted observation before tracker insertion");
            var decision = bridge.acceptRobotNanoseconds(measurement, (BASE_US + 31_000) * 1000);
            check(decision.insertedObservations() == 2, "real admitted observations reach deterministic tracking");
            check(decision.normalization().provenance().lifecycleAdmission() == measurement.admission()
                    && decision.normalization().provenance().clockMapping() == measurement.clock(), "adapter forwards actual results unchanged");
            var snapshot = engine.publish(ego(31_000, 1.05, 2.01, Math.PI / 2));
            check(snapshot.tracks().size() == 2, "two canonical observed objects create two persistent identities");
            check(Math.abs(snapshot.tracks().get(0).positionM().x() - 1) < 1e-9
                    && Math.abs(snapshot.tracks().get(0).positionM().y() - 3) < 1e-9, "capture pose projection differs from current pose");
            check(snapshot.tracks().get(0).lastMeasurementUs() == BASE_US + 20_000, "JSON us and metadata unit profiles yield capture us");
            selectAdmittedTracks(snapshot, pipeline);
            check(pipeline.drain().isEmpty(), "measurement channel consumes once");
            var duplicate = bridge.acceptRobotNanoseconds(measurement, (BASE_US + 31_000) * 1000);
            check(duplicate.tracking().orElseThrow().status() == WorldEngine.FrameStatus.DUPLICATE
                    && duplicate.insertedObservations() == 0, "retained envelope replay cannot double-count measurement");
            pipeline.publish(43, 2, 20_000, 35_000, "fixture-boot-a", true, true);
            check(pipeline.drain().isEmpty(), "republishing same capture never produces a second measurement");
            pipeline.publish(43, 3, 20_000, 40_000, "fixture-boot-a", false, true);
            check(pipeline.drain().isEmpty() && !pipeline.client.statuses(pipeline.nowNs.get()).get(SOURCE).actionable(),
                    "same-frame watchdog clears source before dedup");
            check(bridge.acceptRobotNanoseconds(measurement, pipeline.nowNs.get()).normalization().reason()
                    == CustomVisionAdapter.Reason.NO_NEW_MEASUREMENT, "historical pre-watchdog envelope cannot become deliverable again");
            check(engine.publish(ego(40_000, 1.05, 2.01, Math.PI / 2)).tracks().size() == 2, "unavailable camera is not free space");
        }
        check(closedQueue.closed, "only owned fake queue closed");
    }
    private static void selectAdmittedTracks(World.WorldSnapshot snapshot, Pipeline pipeline) {
        var footprint = new PlannerBackend.Footprint(.4, .4);
        var safety = new SafetySupervisor(new SafetySupervisor.Config(100_000, 100_000,
                20_000, 2, .01, .01, 1, footprint, true, SafetySupervisor.Action.STOP, Set.of("AUTO")));
        var configuration = new TargetSelector.Config(new TargetSelector.Quality(100_000, .5, 1, 1),
                new TargetSelector.CollectionGeometry(.8, new Geometry.Vec2(.5, 0), List.of(0.0), .05, .1, .1),
                new TargetSelector.Ranking(Map.of("synthetic_piece", 1.0), .1, .2, 0),
                new TargetSelector.Planning(footprint, new PlannerBackend.Constraints(2, 3, 4),
                        new PlannerBackend.Bounds(0, 0, 10, 10), 16, 32));
        // Only the reachability backend is synthetic; admission and persistent tracking above are real.
        PlannerBackend backend = request -> new PlannerBackend.Result(request.requestId(), request.taskId(),
                request.epoch(), request.snapshotId(), request.obstacleMapVersion(), PlannerBackend.Status.SUCCESS,
                new PlannerBackend.GeometricPath(List.of(request.startPose(), request.goal().pose())), null, 100,
                request.validUntilUs(), "synthetic-bridge-straight", "CPU synthetic geometry only");
        var selector = new TargetSelector(configuration, safety, backend, () -> pipeline.nowNs.get() / 1000);
        var facts = new World.RobotFacts(snapshot.ego(), true, true, "AUTO", false, true, true, false, true,
                snapshot.ego().timeUs());
        var outcome = selector.select(new TargetSelector.Request("admitted-cluster", "synthetic-pickup",
                TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, facts, snapshot.ego().timeUs(),
                snapshot.ego().timeUs() + 50_000, 100, List.of(), true,
                new PlannerBackend.SearchBudget(System.nanoTime(), 1_000_000_000), () -> false));
        check(snapshot.tracks().stream().allMatch(t -> t.lifecycle() == World.TrackLifecycle.COASTING
                && t.kind() == World.EstimateKind.PROPAGATED), "capture latency preserves propagated coasting distinction");
        check(outcome.status() == TargetSelector.Status.SELECTED && outcome.selection().members().size() == 2,
                "real admitted capture reaches useful persistent-object cluster selection: " + outcome.detail());
        check(outcome.selection().members().stream().allMatch(m -> m.lastMeasurementUs() == BASE_US + 20_000),
                "selection retains capture freshness instead of promoting snapshot time to measurement time");
        check(!facts.possessionVerified(), "selection cannot fabricate physical pickup completion");
    }
    private static void boundsAndEpochs() {
        var version = TimeVersion.WPILIB_2026_MICROSECONDS;
        try (Pipeline pipeline = new Pipeline(version)) {
            var measurement = adopt(pipeline);
            WorldEngine outside = engine();
            outside.addEgo(ego(31_000, 1, 2, 0));
            var oldBridge = new VisionTrackBridge(adapter(measurement.packet(), version, 0, 2), outside, pipeline.client);
            var rejected = oldBridge.acceptRobotNanoseconds(measurement, (BASE_US + 31_000) * 1000);
            check(rejected.normalization().status() == CustomVisionAdapter.Status.ACCEPTED
                    && rejected.tracking().orElseThrow().status() == WorldEngine.FrameStatus.HISTORY_BOUNDS
                    && rejected.insertedObservations() == 0, "accepted normalization does not bypass capture history bounds");
            outside.reset(World.ResetReason.LOCALIZATION_HARD_RESET, FIELD, false, BASE_US + 31_000);
            check(oldBridge.acceptRobotNanoseconds(measurement, (BASE_US + 31_000) * 1000).tracking().orElseThrow().status() == WorldEngine.FrameStatus.WRONG_EPOCH,
                    "old adapter binding cannot insert after a world reset");

            pipeline.publish(44, 0, 50_000, 51_000, "fixture-boot-b", true, true);
            check(pipeline.drain().isEmpty(), "new boot needs advancing adoption");
            pipeline.publish(45, 1, 60_000, 61_000, "fixture-boot-b", true, true);
            var restarted = pipeline.drain().get(0);
            check(oldBridge.acceptRobotNanoseconds(restarted, (BASE_US + 61_000) * 1000).normalization().reason() == CustomVisionAdapter.Reason.BOOT_MISMATCH,
                    "new adopted source cannot authorize an old configured boot binding");
            outside.addEgo(ego(60_000, 1, 2, 0)); outside.addEgo(ego(61_000, 1, 2, 0));
            var rebound = new VisionTrackBridge(adapter(restarted.packet(), version, outside.epoch(), 2), outside, pipeline.client);
            check(rebound.acceptRobotNanoseconds(restarted, (BASE_US + 61_000) * 1000).insertedObservations() == 2, "robot-owned explicit reset and binding rebuild admits new epoch");
            pipeline.publish(46, 9, 70_000, 71_000, "fixture-boot-a", true, true);
            check(pipeline.drain().isEmpty() && pipeline.client.counters().lifecycleRejected() > 0, "retired boot cannot return with advancing sequence");

            pipeline.connection.set(3);
            pipeline.nowNs.set((BASE_US + 80_000) * 1000);
            pipeline.client.disconnect(SOURCE, 3, pipeline.nowNs.get());
            pipeline.publish(47, 2, 81_000, 82_000, "fixture-boot-b", true, true);
            check(pipeline.drain().isEmpty(), "reconnect requires adoption");
            pipeline.publish(48, 3, 84_000, 85_000, "fixture-boot-b", true, true);
            var reconnected = pipeline.drain().get(0);
            check(rebound.acceptRobotNanoseconds(reconnected, (BASE_US + 85_000) * 1000).normalization().reason() == CustomVisionAdapter.Reason.CLOCK_MISMATCH,
                    "old connection binding rejects even a newly admitted reconnect capture");
        }
    }
    private static void mappingAndWatchdogPurges() {
        var version = TimeVersion.WPILIB_2026_MICROSECONDS;
        try (Pipeline pipeline = new Pipeline(version)) {
            pipeline.publish(42, 0, 0, 10_000, "fixture-boot-a", true, false);
            pipeline.publish(43, 1, 20_000, 31_000, "fixture-boot-a", true, false);
            check(pipeline.drain().isEmpty(), "unverified actual golden timing produces no admitted measurement");
            var receipt = pipeline.client.drainReceipts().get(0);
            check(receipt.clock().rejection().orElseThrow().reason() == ClockMapper.Reason.CAPTURE_CORRECTION_UNVERIFIED,
                    "diagnostic receipt preserves rejected clock; never reconstructed into acceptance");
        }
        try (Pipeline pipeline = new Pipeline(version)) {
            pipeline.publish(42, 0, 0, 10_000, "fixture-boot-a", true, true);
            pipeline.publish(43, 1, 20_000, 31_000, "fixture-boot-a", true, true);
            check(pipeline.client.counters().pendingMeasurements() == 1, "actual accepted output waits for consume");
            pipeline.publish(43, 2, 20_000, 40_000, "fixture-boot-a", false, true);
            check(pipeline.drain().isEmpty(), "watchdog purges queued accepted output before consumer drain");
        }
    }
    public static void main(String[] args) throws Exception {
        if (args.length != 1) throw new IllegalArgumentException("producer fixture directory required");
        byte[] bytes = Files.readAllBytes(Path.of(args[0]).resolve("objects_compact_covariance_selection.json"));
        check(HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(bytes))
                .equals("c0e5c0ea9241423e1d3aa8fe19373931cf3fab080ced9af2b9021d9eaf6fc6d9"), "producer golden input pin");
        golden = new String(bytes, java.nio.charset.StandardCharsets.UTF_8);
        consumeIntoTracking(TimeVersion.WPILIB_2026_MICROSECONDS);
        consumeIntoTracking(TimeVersion.WPILIB_2027_ALPHA7_NANOSECONDS);
        boundsAndEpochs(); mappingAndWatchdogPurges();
        System.out.println("AdmittedVisionTrackingTest: " + checks + " checks passed (real admission; synthetic I/O/timing only)");
    }
    private static void check(boolean condition, String detail) { checks++; if (!condition) throw new AssertionError(detail); }
}
