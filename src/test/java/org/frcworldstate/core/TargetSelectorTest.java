package org.frcworldstate.core;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import static org.frcworldstate.core.Geometry.*;

/** Small CPU synthetic checks; these fake planners make no hardware or collection claims. */
public final class TargetSelectorTest {
    private TargetSelectorTest() {}
    private static int checks;
    private static final FieldIdentity FIELD = new FieldIdentity("offseason", "test-map", "geometry-v1");
    private static final PlannerBackend.Footprint FOOTPRINT = new PlannerBackend.Footprint(.6, .6);
    private static final PlannerBackend.Constraints CONSTRAINTS = new PlannerBackend.Constraints(2, 3, 4);
    private static final PlannerBackend.Bounds BOUNDS = new PlannerBackend.Bounds(0, 0, 20, 10);
    private static final long NOW_US = 1_000_000;
    private static final long BUDGET_NANOS = 2_000_000_000L;

    public static void main(String[] args) { run(); }
    public static void run() {
        explicitAndCluster();
        identityAndQuality();
        blockedAndUntrustedApproaches();
        hysteresisAndResets();
        boundedAndMissingBindings();
        clearanceAndFinalFreshness();
        plannerEvidenceExpiry();
        System.out.println("TargetSelectorTest: " + checks + " checks passed (synthetic bounded geometric backends)");
    }
    private static void check(boolean condition, String detail) {
        checks++; if (!condition) throw new AssertionError(detail);
    }
    private static TargetSelector.Config config(double travelCost, double hysteresis, long lockUs, List<Double> angles, int cap) {
        return new TargetSelector.Config(new TargetSelector.Quality(500_000, .7, .01, 1),
            new TargetSelector.CollectionGeometry(.75, new Vec2(.75, 0), angles, .05, .1, .1),
            new TargetSelector.Ranking(Map.of("piece", 1.0, "valuable", 5.0, "ignored", 0.0), travelCost, hysteresis, lockUs),
            new TargetSelector.Planning(FOOTPRINT, CONSTRAINTS, BOUNDS, cap, 32));
    }
    private static TargetSelector.Config config() { return config(.1, .2, 0, List.of(0.0), 256); }
    private static SafetySupervisor safety(PlannerBackend.Footprint footprint) {
        return new SafetySupervisor(new SafetySupervisor.Config(500_000, 500_000, 20_000, 2, .01, .01, 1,
            footprint, true, SafetySupervisor.Action.STOP, Set.of("AUTO")));
    }
    private static World.ObjectTrack track(long id, long epoch, String objectClass, double x, double y, long nowUs) {
        World.SourceStamp stamp = new World.SourceStamp("camera-a", "boot-a", "cal-v1", "mount-v1", "shared-stereo", id,
            nowUs, nowUs, .05);
        return new World.ObjectTrack(id, epoch, objectClass, new Vec2(x, y), new Vec2(0, 0), nowUs, nowUs, .9,
            new Uncertainty(.0004, 0, .0004), World.TrackLifecycle.CONFIRMED, World.EstimateKind.MEASURED, List.of(stamp));
    }
    private static World.ObjectTrack modified(World.ObjectTrack track, long lastMeasurementUs, double confidence,
            Uncertainty covariance, World.TrackLifecycle lifecycle, List<World.SourceStamp> provenance) {
        return new World.ObjectTrack(track.id(), track.epoch(), track.objectClass(), track.positionM(), track.velocityMps(),
            track.estimateUs(), lastMeasurementUs, confidence, covariance, lifecycle, track.kind(), provenance);
    }
    private static World.WorldSnapshot snapshot(long id, long epoch, long map, FieldIdentity field, long nowUs, List<World.ObjectTrack> tracks) {
        World.EgoState ego = new World.EgoState(nowUs, new Pose2(new Vec2(1, 2), 0), Velocity2.zero(), new Uncertainty(.0004, 0, .0004), true);
        return new World.WorldSnapshot(id, epoch, map, field, ego, tracks);
    }
    private static World.RobotFacts facts(World.WorldSnapshot snapshot) {
        return new World.RobotFacts(snapshot.ego(), true, true, "AUTO", false, true, true, false, true, snapshot.ego().timeUs());
    }
    private static TargetSelector.Request request(String task, String requestId, TargetSelector.Mode mode, Long selected,
            World.WorldSnapshot snapshot, List<PlannerBackend.Obstacle> obstacles, boolean coverage,
            PlannerBackend.SearchBudget budget, PlannerBackend.Cancellation cancellation) {
        return new TargetSelector.Request(requestId, task, mode, selected, snapshot, facts(snapshot), snapshot.ego().timeUs(),
            snapshot.ego().timeUs() + 100_000, 100, obstacles, coverage, budget, cancellation);
    }
    private static TargetSelector.Request request(String task, TargetSelector.Mode mode, Long selected, World.WorldSnapshot snapshot) {
        return request(task, task + ":" + snapshot.id(), mode, selected, snapshot, List.of(), true,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
    }
    private static TargetSelector selector(TargetSelector.Config config, PlannerBackend backend, AtomicLong clock) {
        return new TargetSelector(config, safety(FOOTPRINT), backend, clock::get);
    }
    private static PlannerBackend.Result straight(PlannerBackend.Request r) {
        return new PlannerBackend.Result(r.requestId(), r.taskId(), r.epoch(), r.snapshotId(), r.obstacleMapVersion(),
            PlannerBackend.Status.SUCCESS, new PlannerBackend.GeometricPath(List.of(r.startPose(), r.goal().pose())), null, 100,
            r.validUntilUs(), "synthetic-straight", "fake-I/O straight geometric segment; independently validated");
    }
    private static PlannerBackend.Result failure(PlannerBackend.Request r, PlannerBackend.Status status) {
        return new PlannerBackend.Result(r.requestId(), r.taskId(), r.epoch(), r.snapshotId(), r.obstacleMapVersion(),
            status, null, null, 100, r.validUntilUs(), "synthetic-failure", "explicit fake-I/O backend outcome");
    }
    private static TargetSelector.Selection selected(TargetSelector.Outcome outcome, String detail) {
        check(outcome.status() == TargetSelector.Status.SELECTED, detail + ": " + outcome.status() + " " + outcome.detail());
        return outcome.selection();
    }

    private static void explicitAndCluster() {
        AtomicLong clock = new AtomicLong(NOW_US);
        var selector = selector(config(), TargetSelectorTest::straight, clock);
        var ordinary1 = track(1, 0, "piece", 4, 2, NOW_US);
        var ordinary2 = track(2, 0, "piece", 4.3, 2, NOW_US);
        var valuable = track(3, 0, "valuable", 7, 4, NOW_US);
        var snapshot = snapshot(1, 0, 1, FIELD, NOW_US, List.of(ordinary1, ordinary1, ordinary2, valuable));
        var cluster = selected(selector.select(request("pick", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot)), "weighted cluster selects");
        check(cluster.anchor().id() == 3 && cluster.members().size() == 1 && cluster.utility() == 5,
            "greatest useful cluster beats more low-utility detections and exact duplicate track");
        check(cluster.evidence().backendId().equals("synthetic-straight"), "actual injected-backend evidence retained");
        check(cluster.goal().pose().toField(config().geometry().intakePickupPointM()).subtract(cluster.collectionCenterM()).norm() < 1e-12,
            "configured robot intake point maps to collection center exactly once");
        check(cluster.matches(snapshot, "pick", NOW_US) && !cluster.matches(snapshot, "replacement", NOW_US), "task admission identity checked");
        check(!cluster.matches(snapshot(2, 0, 1, FIELD, NOW_US, snapshot.tracks()), "pick", NOW_US), "old snapshot selection rejected");
        check(!cluster.matches(snapshot, "pick", cluster.validUntilUs()), "expiry admission is exclusive");
        var explicit = selected(selector.select(request("explicit", TargetSelector.Mode.SELECTED_OBJECT, 1L, snapshot)), "explicit persistent identity selects");
        check(explicit.anchor().id() == 1 && explicit.members().size() == 1 && explicit.utility() == 1,
            "selected-object mode never silently changes target or reports adjacent cluster collection");
        var nearOnly = snapshot(2, 0, 1, FIELD, NOW_US, List.of(ordinary1, ordinary1, ordinary2));
        var pair = selected(selector.select(request("cluster", TargetSelector.Mode.USEFUL_CLUSTER, null, nearOnly)), "physical pair selects");
        check(pair.members().size() == 2 && pair.utility() == 2, "physical persistent identities count once");
        check(pair.members().get(0).key().id() == 1 && pair.members().get(1).key().id() == 2, "member order deterministic by persistent identity");
        boolean immutable = false;
        try { pair.members().clear(); } catch (UnsupportedOperationException ex) { immutable = true; }
        check(immutable, "immutable selected member list");
        var edge = track(2, 0, "piece", 4.71, 2, NOW_US);
        var edgeResult = selected(selector.select(request("edge", TargetSelector.Mode.USEFUL_CLUSTER, null,
            snapshot(3, 0, 1, FIELD, NOW_US, List.of(ordinary1, edge)))), "conservative disk boundary selects");
        check(edgeResult.members().size() == 1, "physical radius shrinks for configured covariance and goal-position tolerance");
    }

    private static void identityAndQuality() {
        AtomicLong clock = new AtomicLong(NOW_US);
        var base = track(1, 0, "piece", 4, 2, NOW_US);
        var selector = selector(config(), TargetSelectorTest::straight, clock);
        var stale = modified(base, NOW_US - 500_001, .9, base.uncertainty(), base.lifecycle(), base.provenance());
        var staleOutcome = selector.select(request("stale", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot(1, 0, 1, FIELD, NOW_US, List.of(stale))));
        check(staleOutcome.status() == TargetSelector.Status.NO_ELIGIBLE_TRACKS, "fresh estimate cannot freshen stale measurement");
        var low = modified(base, NOW_US, .4, base.uncertainty(), base.lifecycle(), base.provenance());
        var conflict = selector.select(request("conflict", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot(2, 0, 1, FIELD, NOW_US, List.of(low, base))));
        check(conflict.status() == TargetSelector.Status.INVALID_INPUT, "conflicting persistent duplicate cannot rehabilitate bad confidence");
        var tentative = modified(base, NOW_US, .9, base.uncertainty(), World.TrackLifecycle.TENTATIVE, base.provenance());
        var uncertain = modified(base, NOW_US, .9, new Uncertainty(.02, 0, .02), base.lifecycle(), base.provenance());
        var unknown = track(2, 0, "unknown", 5, 2, NOW_US);
        var ignored = track(3, 0, "ignored", 5, 3, NOW_US);
        for (World.ObjectTrack bad : List.of(low, tentative, uncertain, unknown, ignored)) {
            var outcome = selector.select(request("bad-" + checks, TargetSelector.Mode.SELECTED_OBJECT, bad.id(), snapshot(3, 0, 1, FIELD, NOW_US, List.of(bad))));
            check(outcome.status() == TargetSelector.Status.SELECTED_TRACK_UNAVAILABLE, "explicit target fails quality/class/lifecycle gate");
        }
        var captured = track(1, 0, "piece", 4, 2, NOW_US - 5_000);
        var coasting = new World.ObjectTrack(captured.id(), captured.epoch(), captured.objectClass(), captured.positionM(), captured.velocityMps(),
            NOW_US, captured.lastMeasurementUs(), captured.confidence(), captured.uncertainty(), World.TrackLifecycle.COASTING,
            World.EstimateKind.PROPAGATED, captured.provenance());
        var coast = selected(selector.select(request("coasting", TargetSelector.Mode.SELECTED_OBJECT, 1L,
            snapshot(3, 0, 1, FIELD, NOW_US, List.of(coasting)))), "capture-latency coasting remains eligible while measured freshness holds");
        check(coast.members().get(0).lifecycle() == World.TrackLifecycle.COASTING
            && coast.members().get(0).estimateKind() == World.EstimateKind.PROPAGATED
            && coast.members().get(0).estimateUs() == NOW_US && coast.members().get(0).lastMeasurementUs() == NOW_US - 5_000,
            "coasting estimate remains distinct from its older physical observation");
        var staleCoasting = modified(coasting, NOW_US - 500_001, .9, coasting.uncertainty(), coasting.lifecycle(), coasting.provenance());
        check(selector.select(request("stale-coasting", TargetSelector.Mode.SELECTED_OBJECT, 1L,
            snapshot(3, 0, 1, FIELD, NOW_US, List.of(staleCoasting)))).status() == TargetSelector.Status.SELECTED_TRACK_UNAVAILABLE,
            "allowing COASTING does not freshen an old measurement");
        var future = track(1, 0, "piece", 4, 2, NOW_US + 1);
        check(selector.select(request("future", TargetSelector.Mode.SELECTED_OBJECT, 1L, snapshot(4, 0, 1, FIELD, NOW_US, List.of(future)))).status()
            == TargetSelector.Status.SELECTED_TRACK_UNAVAILABLE, "future measurement or estimate rejected");
        var camera2 = new World.SourceStamp("camera-b", "boot-b", "cal-v1", "mount-v1", "shared-stereo", 1, NOW_US, NOW_US, .05);
        var correlated = modified(base, NOW_US, .9, base.uncertainty(), base.lifecycle(), List.of(base.provenance().get(0), camera2));
        var onePhysical = selected(selector.select(request("stereo", TargetSelector.Mode.USEFUL_CLUSTER, null,
            snapshot(5, 0, 1, FIELD, NOW_US, List.of(correlated, correlated)))), "correlated observation provenance selects once");
        check(onePhysical.utility() == 1 && onePhysical.members().get(0).provenance().size() == 2,
            "correlated cameras are provenance, never two independent collection utilities");
        var missing = selector.select(request("absent", TargetSelector.Mode.SELECTED_OBJECT, 99L, snapshot(6, 0, 1, FIELD, NOW_US, List.of(base))));
        check(missing.status() == TargetSelector.Status.SELECTED_TRACK_UNAVAILABLE, "absent explicit target abstains instead of replacing it");
    }

    private static void blockedAndUntrustedApproaches() {
        AtomicLong clock = new AtomicLong(NOW_US);
        var track = track(1, 0, "piece", 4, 2, NOW_US);
        var snapshot = snapshot(1, 0, 1, FIELD, NOW_US, List.of(track));
        var obstacle = new PlannerBackend.Obstacle("blocked-nearest", new Vec2(3.25, 2), .07, 0, NOW_US + 200_000, false);
        var req = request("blocked", "blocked:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(obstacle), true,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
        var blocked = selector(config(), TargetSelectorTest::straight, clock).select(req);
        check(blocked.status() == TargetSelector.Status.NO_REACHABLE_APPROACH, "blocked collection approach never yields a goal");
        check(blocked.evaluations().get(0).backendId().equals("core-geometry"), "goal footprint collision filtered independently before search");
        var alternative = selected(selector(config(.1, 0, 0, List.of(0.0, Math.PI / 2), 256), TargetSelectorTest::straight, clock).select(req),
            "configured alternate safe approach searched");
        check(Math.abs(angle(alternative.goal().pose().headingRad() - Math.PI / 2)) < 1e-12,
            "blocked nearest approach replaced only by validated configured alternate");
        var segmentObstacle = new PlannerBackend.Obstacle("mid-path", new Vec2(2, 2), .1, 0, NOW_US + 200_000, false);
        var segment = request("segment", "segment:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(segmentObstacle), true,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
        var untrusted = selector(config(), TargetSelectorTest::straight, clock).select(segment);
        check(untrusted.status() == TargetSelector.Status.NO_REACHABLE_APPROACH && untrusted.evaluations().get(0).status() == PlannerBackend.Status.INVALID_INPUT,
            "planner SUCCESS with unsafe intermediate geometry independently rejected");
        var noPath = selector(config(), r -> failure(r, PlannerBackend.Status.NO_PATH), clock).select(request("no-path", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(noPath.status() == TargetSelector.Status.NO_REACHABLE_APPROACH, "NO_PATH is abstention, never target success");
        var nullResult = selector(config(), r -> null, clock).select(request("null", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(nullResult.status() == TargetSelector.Status.NO_REACHABLE_APPROACH && nullResult.evaluations().get(0).status() == PlannerBackend.Status.INVALID_INPUT,
            "absent backend proof is not reachable");
        PlannerBackend oldReply = r -> new PlannerBackend.Result("old-request", r.taskId(), r.epoch(), r.snapshotId(), r.obstacleMapVersion(),
            PlannerBackend.Status.SUCCESS, new PlannerBackend.GeometricPath(List.of(r.startPose(), r.goal().pose())), null, 100, r.validUntilUs(), "late-backend", "old reply");
        var stale = selector(config(), oldReply, clock).select(request("old-reply", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(stale.status() == TargetSelector.Status.NO_REACHABLE_APPROACH && stale.evaluations().get(0).status() == PlannerBackend.Status.STALE_RESULT,
            "late asynchronous planner identity rejected");
        PlannerBackend timed = r -> new PlannerBackend.Result(r.requestId(), r.taskId(), r.epoch(), r.snapshotId(), r.obstacleMapVersion(), PlannerBackend.Status.SUCCESS,
            new PlannerBackend.GeometricPath(List.of(r.startPose(), r.goal().pose())), new PlannerBackend.TimedTrajectory(List.of(
                new PlannerBackend.TimedPoint(0, r.startPose(), Velocity2.zero()), new PlannerBackend.TimedPoint(1, r.goal().pose(), Velocity2.zero()))),
            100, r.validUntilUs(), "unqualified-timed", "unsupported timed geometry");
        var timedOutcome = selector(config(), timed, clock).select(request("timed", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(timedOutcome.status() == TargetSelector.Status.NO_REACHABLE_APPROACH, "unqualified timed-trajectory proof rejected");
        PlannerBackend throwing = r -> { throw new IllegalStateException("synthetic failure"); };
        var thrown = selector(config(), throwing, clock).select(request("throws", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(thrown.status() == TargetSelector.Status.NO_REACHABLE_APPROACH && thrown.evaluations().get(0).backendId().equals("backend-exception"),
            "backend failure is observable abstention");
    }

    private static void hysteresisAndResets() {
        AtomicLong clock = new AtomicLong(NOW_US);
        var selector = selector(config(1, .75, 0, List.of(0.0), 256), TargetSelectorTest::straight, clock);
        var first = snapshot(1, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "piece", 4, 2, NOW_US), track(2, 0, "piece", 4, 3.5, NOW_US)));
        var a = selected(selector.select(request("stable", TargetSelector.Mode.USEFUL_CLUSTER, null, first)), "initial hysteresis selection");
        check(a.anchor().id() == 1 && !a.retainedByHysteresis(), "initial best anchor chosen");
        clock.set(NOW_US + 1_000);
        var second = snapshot(2, 0, 1, FIELD, clock.get(), List.of(track(1, 0, "piece", 4.2, 2, clock.get()), track(2, 0, "piece", 3.4, 3.5, clock.get())));
        var retained = selected(selector.select(request("stable", TargetSelector.Mode.USEFUL_CLUSTER, null, second)), "small score change retains lock");
        check(retained.anchor().id() == 1 && retained.retainedByHysteresis(), "configured switching-improvement hysteresis prevents churn");
        clock.set(NOW_US + 2_000);
        var third = snapshot(3, 0, 1, FIELD, clock.get(), List.of(track(1, 0, "piece", 5.5, 2, clock.get()), track(2, 0, "piece", 3.4, 3.5, clock.get())));
        var switched = selected(selector.select(request("stable", TargetSelector.Mode.USEFUL_CLUSTER, null, third)), "meaningful score improvement switches");
        check(switched.anchor().id() == 2 && !switched.retainedByHysteresis(), "large utility/cost improvement replaces old target");
        check(selector.select(request("stable", TargetSelector.Mode.USEFUL_CLUSTER, null, second)).status() == TargetSelector.Status.STALE_REQUEST,
            "older queued snapshot cannot poison hysteresis lock");
        var replacement = selected(selector.select(request("new-task", TargetSelector.Mode.SELECTED_OBJECT, 1L, third)), "task replacement selects requested track");
        check(replacement.anchor().id() == 1 && !replacement.retainedByHysteresis(), "task/mode replacement clears lock");
        var epoch = snapshot(4, 1, 1, FIELD, clock.get(), List.of(track(1, 1, "piece", 4, 2, clock.get())));
        var changedEpoch = selected(selector.select(request("new-task", TargetSelector.Mode.SELECTED_OBJECT, 1L, epoch)), "epoch reset accepts rebuilt identity");
        check(changedEpoch.anchor().epoch() == 1 && !changedEpoch.retainedByHysteresis(), "epoch reset clears reused numeric track identity");
        check(!replacement.matches(epoch, "new-task", clock.get()), "old epoch worker reply inadmissible");
        var map = snapshot(5, 1, 2, FIELD, clock.get(), epoch.tracks());
        var changedMap = selected(selector.select(request("new-task", TargetSelector.Mode.SELECTED_OBJECT, 1L, map)), "map change selects afresh");
        check(!changedMap.retainedByHysteresis(), "changed obstacle map clears lock");
        var field = snapshot(6, 1, 2, new FieldIdentity("offseason", "test-map", "geometry-v2"), clock.get(), epoch.tracks());
        check(!selected(selector.select(request("new-task", TargetSelector.Mode.SELECTED_OBJECT, 1L, field)), "geometry change selects afresh").retainedByHysteresis(),
            "fixed field/geometry identity change clears lock");
        selector.reset();
        check(!selected(selector.select(request("new-task", TargetSelector.Mode.SELECTED_OBJECT, 1L, field)), "explicit reset selects afresh").retainedByHysteresis(), "explicit cancellation resets lock");

        clock.set(NOW_US);
        var minimumLock = selector(config(1, 0, 100_000, List.of(0.0), 256), TargetSelectorTest::straight, clock);
        selected(minimumLock.select(request("hold-time", TargetSelector.Mode.USEFUL_CLUSTER, null, first)), "minimum-lock initial selection");
        clock.set(NOW_US + 1_000);
        var extra = snapshot(2, 0, 1, FIELD, clock.get(), List.of(track(1, 0, "piece", 4, 2, clock.get()),
            track(2, 0, "piece", 4, 3.5, clock.get()), track(3, 0, "valuable", 4, 3.6, clock.get())));
        check(selected(minimumLock.select(request("hold-time", TargetSelector.Mode.USEFUL_CLUSTER, null, extra)), "minimum time suppresses improvement").anchor().id() == 1,
            "configured minimum lock interval honored while old anchor stays reachable");
        clock.set(NOW_US + 100_001);
        var later = snapshot(3, 0, 1, FIELD, clock.get(), List.of(track(1, 0, "piece", 4, 2, clock.get()),
            track(2, 0, "piece", 4, 3.5, clock.get()), track(3, 0, "valuable", 4, 3.6, clock.get())));
        check(selected(minimumLock.select(request("hold-time", TargetSelector.Mode.USEFUL_CLUSTER, null, later)), "minimum time elapsed switches").anchor().id() == 2,
            "best configured useful cluster becomes eligible after lock interval");
    }

    private static void boundedAndMissingBindings() {
        AtomicLong clock = new AtomicLong(NOW_US);
        var snapshot = snapshot(1, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "piece", 4, 2, NOW_US), track(2, 0, "piece", 6, 3, NOW_US)));
        var req = request("bindings", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot);
        check(new TargetSelector(null, safety(FOOTPRINT), TargetSelectorTest::straight, clock::get).select(req).status() == TargetSelector.Status.MISSING_CONFIGURATION,
            "missing collection/config geometry fails clearly");
        check(new TargetSelector(config(), null, TargetSelectorTest::straight, clock::get).select(req).status() == TargetSelector.Status.MISSING_SAFETY_SUPERVISOR,
            "missing independent safety fails clearly");
        check(new TargetSelector(config(), safety(FOOTPRINT), null, clock::get).select(req).status() == TargetSelector.Status.MISSING_PLANNER,
            "missing reachability backend cannot be replaced by distance-only confidence");
        check(new TargetSelector(config(), safety(FOOTPRINT), TargetSelectorTest::straight, null).select(req).status() == TargetSelector.Status.MISSING_CLOCK,
            "missing robot monotonic clock fails clearly");
        var base = config();
        var missingConstraints = new TargetSelector.Config(base.quality(), base.geometry(), base.ranking(), new TargetSelector.Planning(FOOTPRINT, null, BOUNDS, 256, 32));
        check(selector(missingConstraints, TargetSelectorTest::straight, clock).select(req).status() == TargetSelector.Status.MISSING_CONFIGURATION, "missing motion limits fails clearly");
        check(new TargetSelector(config(), safety(new PlannerBackend.Footprint(.8, .8)), TargetSelectorTest::straight, clock::get).select(req).status() == TargetSelector.Status.INVALID_INPUT,
            "safety/planning footprint mismatch rejected");
        var unknownCoverage = request("coverage", "coverage:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(), false,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
        check(selector(config(), TargetSelectorTest::straight, clock).select(unknownCoverage).status() == TargetSelector.Status.UNSAFE,
            "empty camera detections do not establish free collision coverage");
        var expiredEnvelope = request("envelope", "envelope:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot,
            List.of(new PlannerBackend.Obstacle("short", new Vec2(10, 8), .1, 0, NOW_US + 99_999, true)), true,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
        check(selector(config(), TargetSelectorTest::straight, clock).select(expiredEnvelope).status() == TargetSelector.Status.UNSAFE,
            "future envelopes must cover complete requested interval");
        AtomicInteger evaluations = new AtomicInteger();
        var limited = selector(config(.1, 0, 0, List.of(0.0), 1), r -> { evaluations.incrementAndGet(); return straight(r); }, clock).select(req);
        check(limited.status() == TargetSelector.Status.CAPACITY_LIMIT && evaluations.get() == 0, "candidate capacity rejects before partial greatest-cluster search");
        var cancelled = request("cancelled", "cancelled:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(), true,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> true);
        check(selector(config(), TargetSelectorTest::straight, clock).select(cancelled).status() == TargetSelector.Status.CANCELLED, "initial cancellation abstains");
        AtomicBoolean cancel = new AtomicBoolean();
        var midCancel = request("mid-cancel", "mid-cancel:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(), true,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), cancel::get);
        check(selector(config(), r -> { cancel.set(true); return straight(r); }, clock).select(midCancel).status() == TargetSelector.Status.CANCELLED,
            "cancellation after backend returns rejects even reachable geometry");
        var expiredBudget = request("budget", "budget:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(), true,
            new PlannerBackend.SearchBudget(System.nanoTime() - 1_000_000, 1), () -> false);
        check(selector(config(), TargetSelectorTest::straight, clock).select(expiredBudget).status() == TargetSelector.Status.TIMEOUT, "expired monotonic shared budget abstains");
        var futureBudget = request("future-budget", "future-budget:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(), true,
            new PlannerBackend.SearchBudget(System.nanoTime() + BUDGET_NANOS, BUDGET_NANOS), () -> false);
        check(selector(config(), TargetSelectorTest::straight, clock).select(futureBudget).status() == TargetSelector.Status.INVALID_INPUT,
            "future search clock cannot silently extend the bound");
        AtomicInteger count = new AtomicInteger();
        var partial = selector(config(), r -> count.incrementAndGet() == 1 ? straight(r) : failure(r, PlannerBackend.Status.TIMEOUT), clock)
            .select(request("partial", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(partial.status() == TargetSelector.Status.TIMEOUT && partial.selection() == null && count.get() == 2, "later timeout discards partial best rather than claiming greatest");
        var expiry = selector(config(), r -> { clock.set(r.validUntilUs()); return straight(r); }, clock)
            .select(request("deadline", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(expiry.status() == TargetSelector.Status.TIMEOUT, "robot request expiry checked separately from nanosecond search budget");
        clock.set(NOW_US);
        var disabled = new World.RobotFacts(snapshot.ego(), false, true, "AUTO", false, true, true, false, true, NOW_US);
        var disabledRequest = new TargetSelector.Request("disabled", "disabled", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, disabled,
            NOW_US, NOW_US + 100_000, 100, List.of(), true, new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
        check(selector(config(), TargetSelectorTest::straight, clock).select(disabledRequest).status() == TargetSelector.Status.UNSAFE, "disable precludes geometric selection for motion");
        var driverCancel = new World.RobotFacts(snapshot.ego(), true, true, "AUTO", true, true, true, false, true, NOW_US);
        var cancelRequest = new TargetSelector.Request("driver", "driver", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, driverCancel,
            NOW_US, NOW_US + 100_000, 100, List.of(), true, new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
        check(selector(config(), TargetSelectorTest::straight, clock).select(cancelRequest).status() == TargetSelector.Status.UNSAFE, "driver cancel precludes geometric selection");
    }

    private static void clearanceAndFinalFreshness() {
        AtomicLong clock = new AtomicLong(NOW_US);
        var snapshot = snapshot(1, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "piece", 4, 2, NOW_US)));
        var distant = new PlannerBackend.Obstacle("distant", new Vec2(15, 8), .2, .15, NOW_US + 200_000, false);
        var req = request("margin", "margin:1", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot, List.of(distant), true,
            new PlannerBackend.SearchBudget(System.nanoTime(), BUDGET_NANOS), () -> false);
        double margin = .01 + Math.sqrt(snapshot.ego().uncertainty().maxVariance());
        PlannerBackend inspect = r -> {
            check(r.footprint().equals(FOOTPRINT), "physical robot dimensions retained without double inflation");
            check(Math.abs(r.obstacles().get(0).uncertaintyMarginM() - (.15 + margin)) < 1e-12,
                "configured current-ego uncertainty/extra margin added exactly once to obstacle envelope");
            check(Math.abs(r.bounds().minXM() - margin) < 1e-12 && Math.abs(r.bounds().maxXM() - (20 - margin)) < 1e-12,
                "field bounds shrink by the same margin exactly once");
            return straight(r);
        };
        var selection = selected(selector(config(), inspect, clock).select(req), "derived conservative robot clearance selects");
        check(Math.abs(selection.robotClearanceMarginM() - margin) < 1e-12 && selection.effectivePlanningBounds().minXM() == margin,
            "effective robot clearance policy retained in selection provenance");
        SafetySupervisor shortFresh = new SafetySupervisor(new SafetySupervisor.Config(50_000, 70_000, 20_000, 2, .01, .01, 1,
            FOOTPRINT, true, SafetySupervisor.Action.STOP, Set.of("AUTO")));
        var capped = selected(new TargetSelector(config(), shortFresh, TargetSelectorTest::straight, clock::get)
            .select(request("fresh-expiry", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot)), "authoritative freshness expiry caps selection");
        check(capped.validUntilUs() == NOW_US + 50_000 && !capped.matches(snapshot, "fresh-expiry", NOW_US + 50_000),
            "selection cannot outlive ego or required-sensing freshness");
        var lateSafety = new TargetSelector(config(), shortFresh, r -> { clock.set(NOW_US + 50_001); return straight(r); }, clock::get)
            .select(request("late-safety", TargetSelector.Mode.USEFUL_CLUSTER, null, snapshot));
        check(lateSafety.status() == TargetSelector.Status.UNSAFE && lateSafety.detail().contains("after evaluation"),
            "safety rechecked after backend work before publication");
        clock.set(NOW_US);
        var nearExpiry = modified(snapshot.tracks().get(0), NOW_US - 499_990, .9,
            snapshot.tracks().get(0).uncertainty(), World.TrackLifecycle.CONFIRMED, snapshot.tracks().get(0).provenance());
        var lateTrackSnapshot = snapshot(2, 0, 1, FIELD, NOW_US, List.of(nearExpiry));
        var lateTrack = selector(config(), r -> { clock.set(NOW_US + 11); return straight(r); }, clock)
            .select(request("aged-measurement", TargetSelector.Mode.USEFUL_CLUSTER, null, lateTrackSnapshot));
        check(lateTrack.status() == TargetSelector.Status.STALE_REQUEST, "worker cannot publish a member that aged out during planning");
        clock.set(NOW_US);
        var overflowing = snapshot(3, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "piece", 1e308, 2, NOW_US), track(2, 0, "piece", -1e308, 2, NOW_US)));
        check(selector(config(), TargetSelectorTest::straight, clock).select(request("overflow", TargetSelector.Mode.USEFUL_CLUSTER, null, overflowing)).status()
            == TargetSelector.Status.INVALID_INPUT, "extreme finite geometry overflow is explicit abstention");
    }

    private static PlannerBackend.Result shortEvidence(PlannerBackend.Request request, long expiryUs) {
        return new PlannerBackend.Result(request.requestId(), request.taskId(), request.epoch(), request.snapshotId(), request.obstacleMapVersion(),
            PlannerBackend.Status.SUCCESS, new PlannerBackend.GeometricPath(List.of(request.startPose(), request.goal().pose())), null,
            100, expiryUs, "synthetic-short-evidence", "bounded geometry evidence expires before the enclosing request");
    }

    private static void plannerEvidenceExpiry() {
        AtomicLong clock = new AtomicLong(NOW_US);
        var one = snapshot(1, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "valuable", 4, 2, NOW_US)));
        var shortReply = selected(selector(config(), r -> shortEvidence(r, NOW_US + 25_000), clock)
            .select(request("short-evidence", TargetSelector.Mode.USEFUL_CLUSTER, null, one)), "shorter backend validity caps selection");
        check(shortReply.validUntilUs() == NOW_US + 25_000 && shortReply.evidence().validUntilUs() == NOW_US + 25_000,
            "selection and provenance preserve the shorter planner-result expiry");
        check(!shortReply.matches(one, "short-evidence", NOW_US + 25_000), "expired shorter planner evidence cannot be admitted");
        var two = snapshot(2, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "valuable", 4, 2, NOW_US), track(2, 0, "piece", 6, 4, NOW_US)));
        AtomicInteger calls = new AtomicInteger();
        PlannerBackend laterEvaluation = r -> {
            if (calls.incrementAndGet() == 1) return shortEvidence(r, NOW_US + 10);
            clock.set(NOW_US + 11);
            return straight(r);
        };
        var expiredBest = selector(config(), laterEvaluation, clock).select(request("expires-later", TargetSelector.Mode.USEFUL_CLUSTER, null, two));
        check(expiredBest.status() == TargetSelector.Status.STALE_REQUEST && expiredBest.selection() == null && calls.get() == 2,
            "previously best geometric evidence expiring during a later candidate call is rejected");
        clock.set(NOW_US);
        AtomicBoolean shorten = new AtomicBoolean();
        PlannerBackend expiresLocked = r -> {
            if (!shorten.get()) return straight(r);
            if (r.requestId().contains(":reach:1:")) return shortEvidence(r, NOW_US + 10);
            clock.set(NOW_US + 11);
            return straight(r);
        };
        var lockedSelector = selector(config(.1, 0, 100_000, List.of(0.0), 256), expiresLocked, clock);
        var original = snapshot(1, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "piece", 4, 2, NOW_US), track(2, 0, "piece", 6, 4, NOW_US)));
        var oldLock = selected(lockedSelector.select(request("locked-evidence", TargetSelector.Mode.USEFUL_CLUSTER, null, original)), "initial evidence lock");
        check(oldLock.anchor().id() == 1, "first reachable physical anchor is locked");
        shorten.set(true);
        var improved = snapshot(2, 0, 1, FIELD, NOW_US, List.of(track(1, 0, "piece", 4, 2, NOW_US), track(2, 0, "valuable", 6, 4, NOW_US)));
        var expiredLock = lockedSelector.select(request("locked-evidence", TargetSelector.Mode.USEFUL_CLUSTER, null, improved));
        check(expiredLock.status() == TargetSelector.Status.STALE_REQUEST && expiredLock.selection() == null,
            "hysteresis cannot rehabilitate selected locked evidence that expired during later work");
    }
}
