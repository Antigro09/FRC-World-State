package org.frcworldstate.planner;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import org.frcworldstate.core.PlannerBackend;
import org.frcworldstate.core.PlannerValidation;
import org.frcworldstate.core.PoseHistory;
import org.frcworldstate.core.SafetySupervisor;
import org.frcworldstate.core.TargetSelector;
import org.frcworldstate.core.World;
import org.frcworldstate.core.WorldEngine;
import pathplanning.backend.AStarPlannerBackend;
import static org.frcworldstate.core.Geometry.*;

/** Small CPU-only cross-repository checks with the real geometric A* backend. */
public final class TargetSelectionIntegrationTest {
    private TargetSelectionIntegrationTest() {}
    private static int checks;
    private static final long NOW_US = 1_000_000;
    private static final long EXPIRY_US = NOW_US + 100_000;
    private static final FieldIdentity FIELD = new FieldIdentity("offseason", "selector-synthetic", "geometry-v1");
    private static final PlannerBackend.Footprint FOOTPRINT = new PlannerBackend.Footprint(.4, .4);
    private static final PlannerBackend.Constraints CONSTRAINTS = new PlannerBackend.Constraints(2, 3, 4);
    private static final PlannerBackend.Bounds BOUNDS = new PlannerBackend.Bounds(0, 0, 8, 5);

    private static void check(boolean condition, String detail) {
        checks++; if (!condition) throw new AssertionError(detail);
    }
    private static TargetSelector.Config config() {
        return new TargetSelector.Config(new TargetSelector.Quality(100_000, .7, .01, 1),
            new TargetSelector.CollectionGeometry(.7, new Vec2(.6, 0), List.of(0.0), .02, .1, .1),
            new TargetSelector.Ranking(Map.of("piece", 1.0, "valuable", 2.5), .02, .1, 0),
            new TargetSelector.Planning(FOOTPRINT, CONSTRAINTS, BOUNDS, 32, 16));
    }
    private static SafetySupervisor safety() {
        return new SafetySupervisor(new SafetySupervisor.Config(200_000, 200_000, 20_000,
            2, .01, .01, 1, FOOTPRINT, true, SafetySupervisor.Action.STOP, Set.of("AUTO")));
    }
    private static World.EgoState ego(long timeUs) {
        return new World.EgoState(timeUs, new Pose2(new Vec2(.6, 2), 0), Velocity2.zero(),
            new Uncertainty(.0001, 0, .0001), true);
    }
    private static World.RobotFacts facts(World.WorldSnapshot snapshot) {
        return new World.RobotFacts(snapshot.ego(), true, true, "AUTO", false, true, true,
            false, true, snapshot.ego().timeUs());
    }
    private static World.ObjectTrack track(long id, String objectClass, double x, double y) {
        var stamp = new World.SourceStamp("camera-a", "boot-a", "cal-v1", "mount-v1", "shared-stereo",
            id, NOW_US, NOW_US, .05);
        return new World.ObjectTrack(id, 0, objectClass, new Vec2(x, y), new Vec2(0, 0), NOW_US, NOW_US,
            .9, new Uncertainty(.0001, 0, .0001), World.TrackLifecycle.CONFIRMED,
            World.EstimateKind.MEASURED, List.of(stamp));
    }
    private static World.WorldSnapshot snapshot(List<World.ObjectTrack> tracks) {
        return new World.WorldSnapshot(1, 0, 1, FIELD, ego(NOW_US), tracks);
    }
    private static TargetSelector.Request request(String name, TargetSelector.Mode mode, Long selected,
            World.WorldSnapshot snapshot, List<PlannerBackend.Obstacle> obstacles, PlannerBackend.Cancellation cancellation) {
        return new TargetSelector.Request(name, "pickup-task", mode, selected, snapshot, facts(snapshot),
            NOW_US, EXPIRY_US, 100, obstacles, true,
            new PlannerBackend.SearchBudget(System.nanoTime(), 2_000_000_000L), cancellation);
    }
    private static TargetSelector.Selection selected(TargetSelector.Outcome outcome, String detail) {
        check(outcome.status() == TargetSelector.Status.SELECTED,
            detail + ": " + outcome.status() + " " + outcome.detail() + " " + outcome.evaluations());
        return outcome.selection();
    }
    private static final class RecordingBackend implements PlannerBackend {
        private final AStarPlannerBackend actual;
        private final List<Request> requests = new ArrayList<>();
        private final List<Result> results = new ArrayList<>();
        private RecordingBackend(AtomicLong robotClock) {
            actual = new AStarPlannerBackend(new AStarPlannerBackend.Options(.1, .02,
                10_000, 10_000, 20_000, true), System::nanoTime, robotClock::get);
        }
        @Override public Result plan(Request request) {
            requests.add(request); Result result = actual.plan(request); results.add(result); return result;
        }
    }
    private static TargetSelector selector(PlannerBackend backend, AtomicLong clock) {
        return new TargetSelector(config(), safety(), backend, clock::get);
    }

    public static void main(String[] args) {
        explicitAndCluster();
        latencyAndCorrelatedCameras();
        detourAndBlockedApproaches();
        cancellationAndPureGeometry();
        System.out.println("TargetSelectionIntegrationTest: " + checks
            + " real A*/selector checks passed; CPU synthetic only, no follower or hardware");
    }

    private static void explicitAndCluster() {
        var clock = new AtomicLong(NOW_US);
        var backend = new RecordingBackend(clock);
        var first = track(1, "piece", 5.5, 3.3);
        var second = track(2, "piece", 5.7, 3.3);
        var third = track(3, "piece", 5.6, 3.55);
        var valuable = track(4, "valuable", 3.8, 1);
        var world = snapshot(List.of(first, first, second, third, valuable));
        var explicit = selected(selector(backend, clock).select(request("selected-object",
            TargetSelector.Mode.SELECTED_OBJECT, 4L, world, List.of(), () -> false)), "persistent object id reaches real A*");
        check(explicit.anchor().id() == 4 && explicit.members().size() == 1,
            "explicit selected-object mode must preserve the requested physical identity");
        check(explicit.evidence().backendId().equals(AStarPlannerBackend.BACKEND_ID),
            "reachability evidence must name the real backend");
        var cluster = selected(selector(backend, clock).select(request("useful-cluster",
            TargetSelector.Mode.USEFUL_CLUSTER, null, world, List.of(), () -> false)), "weighted cluster reaches real A*");
        check(cluster.members().size() == 3 && Math.abs(cluster.utility() - 3) < 1e-12,
            "exact duplicate track must not double-count physical-object utility");
        check(cluster.members().stream().map(m -> m.key().id()).distinct().count() == 3,
            "cluster membership contains unique persistent identities");
        check(cluster.anchor().id() != valuable.id(), "three useful pieces outrank configured single-object utility 2.5");
        check(cluster.matches(world, "pickup-task", NOW_US) && !cluster.matches(world, "replaced-task", NOW_US),
            "selection reply must bind the current snapshot and task identity");
        check(!cluster.matches(new World.WorldSnapshot(2, world.epoch(), world.obstacleMapVersion(), world.field(),
            world.ego(), world.tracks()), "pickup-task", NOW_US), "new snapshot requires selection re-admission");
    }

    private static World.ObservationFrame observed(String source, long sequence, long captureUs, long publicationUs) {
        var stamp = new World.SourceStamp(source, "boot-a", "cal-v1", "mount-v1", "shared-stereo",
            sequence, captureUs, publicationUs, .05);
        return new World.ObservationFrame(0, stamp, List.of(
            new World.Observation("one", "piece", .9, new Vec2(5.4, 0), new Uncertainty(.0001, 0, .0001)),
            new World.Observation("two", "piece", .9, new Vec2(5.4, .35), new Uncertainty(.0001, 0, .0001))));
    }
    private static void latencyAndCorrelatedCameras() {
        var history = new PoseHistory(16, 500_000);
        var engine = new WorldEngine(new WorldEngine.Config(16, 64, 8, 100_000, 300_000,
            10_000, 10_000, .5, .7, .01, .001, 2, 2, 0, 10, .02, 1, .5, .1, false), history, FIELD);
        engine.addEgo(ego(900_000)); engine.addEgo(ego(980_000));
        check(engine.accept(observed("camera-a", 1, 900_000, 910_000), 910_000).acceptedMeasurements() == 2,
            "first actual frame creates two tentative physical tracks");
        check(engine.accept(observed("camera-a", 2, 980_000, 990_000), 990_000).acceptedMeasurements() == 2,
            "second actual frame confirms the two persistent tracks");
        check(engine.accept(observed("camera-b", 0, 980_000, 995_000), 995_000).acceptedMeasurements() == 0,
            "correlated same-capture camera must not count as another physical measurement");
        var world = engine.publish(ego(NOW_US));
        check(world.tracks().size() == 2 && world.tracks().stream().allMatch(t ->
            t.lifecycle() == World.TrackLifecycle.COASTING && t.kind() == World.EstimateKind.PROPAGATED),
            "normal capture-to-control latency produces fresh coasting estimates");
        check(world.tracks().stream().allMatch(t -> t.lastMeasurementUs() == 980_000 && t.estimateUs() == NOW_US
            && t.provenance().get(t.provenance().size() - 1).publicationUs() == 990_000),
            "capture, publication and propagated estimate times remain distinct");
        var clock = new AtomicLong(NOW_US);
        var backend = new RecordingBackend(clock);
        var cluster = selected(selector(backend, clock).select(request("fresh-coasting",
            TargetSelector.Mode.USEFUL_CLUSTER, null, world, List.of(), () -> false)),
            "fresh confirmed-history coasting tracks remain usable under explicit age and uncertainty gates");
        check(cluster.members().size() == 2 && cluster.utility() == 2,
            "correlated observations do not inflate useful physical-object cluster score");
        check(cluster.members().stream().allMatch(m -> m.lifecycle() == World.TrackLifecycle.COASTING
            && m.estimateKind() == World.EstimateKind.PROPAGATED && m.estimateUs() == NOW_US && m.lastMeasurementUs() == 980_000),
            "selection provenance preserves propagated coasting separately from measured capture");
        check(cluster.validUntilUs() <= 1_080_000,
            "fresh coasting selection expires at its measured capture freshness cutoff");
        var stale = new World.ObjectTrack(30, 0, "valuable", new Vec2(6, 2), new Vec2(0, 0), NOW_US,
            899_999, .9, new Uncertainty(.0001, 0, .0001), World.TrackLifecycle.COASTING,
            World.EstimateKind.PROPAGATED, List.of());
        var staleOutcome = selector(backend, clock).select(request("stale-coasting", TargetSelector.Mode.SELECTED_OBJECT,
            30L, snapshot(List.of(stale)), List.of(), () -> false));
        check(staleOutcome.status() == TargetSelector.Status.SELECTED_TRACK_UNAVAILABLE && staleOutcome.selection() == null,
            "coasting does not grant freshness to an old last measurement");
    }

    private static void detourAndBlockedApproaches() {
        var clock = new AtomicLong(NOW_US);
        var backend = new RecordingBackend(clock);
        var world = snapshot(List.of(track(1, "piece", 6, 2)));
        var obstacle = new PlannerBackend.Obstacle("middle", new Vec2(3, 2), .45, .15, EXPIRY_US, false);
        var selection = selected(selector(backend, clock).select(request("obstacle-detour", TargetSelector.Mode.SELECTED_OBJECT,
            1L, world, List.of(obstacle), () -> false)), "real A* can route around a conservative envelope");
        var plan = backend.requests.get(0); var result = backend.results.get(0);
        check(result.status() == PlannerBackend.Status.SUCCESS && result.trajectory() == null,
            "real backend returns geometric success without timing");
        check(result.path().points().stream().anyMatch(p -> Math.abs(p.position().y() - 2) > .8),
            "route must detour around the inflated middle obstacle");
        check(selection.travelDistanceM() > 4.8, "geometric route length reflects detour rather than Euclidean target distance");
        check(PlannerValidation.validate(plan, result, world, world.epoch(), world.obstacleMapVersion(), NOW_US).status()
            == PlannerBackend.Status.SUCCESS, "owner validator independently accepts real A* detour");
        check(plan.footprint().equals(FOOTPRINT) && Math.abs(plan.obstacles().get(0).uncertaintyMarginM()
            - (obstacle.uncertaintyMarginM() + selection.robotClearanceMarginM())) < 1e-12,
            "selector adds its ego margin once while retaining raw physical footprint");
        check(selection.effectivePlanningBounds().equals(plan.bounds()), "effective bounds remain in selection provenance");

        var blockedGoalBackend = new RecordingBackend(clock);
        var blockedGoal = new PlannerBackend.Obstacle("pickup-obstructed", new Vec2(5.4, 2), .3, 0, EXPIRY_US, false);
        var goalOutcome = selector(blockedGoalBackend, clock).select(request("blocked-approach", TargetSelector.Mode.SELECTED_OBJECT,
            1L, world, List.of(blockedGoal), () -> false));
        check(goalOutcome.status() == TargetSelector.Status.NO_REACHABLE_APPROACH && goalOutcome.selection() == null,
            "unsafe intake approach must not become a selected target");
        check(blockedGoalBackend.requests.isEmpty(), "known colliding goal is rejected before A* work");

        var corridorBackend = new RecordingBackend(clock);
        var wall = new PlannerBackend.Obstacle("closed-corridor", new Vec2(3, 2.5), 2.1, 0, EXPIRY_US, false);
        var corridor = selector(corridorBackend, clock).select(request("closed-corridor", TargetSelector.Mode.SELECTED_OBJECT,
            1L, world, List.of(wall), () -> false));
        check(corridor.status() == TargetSelector.Status.NO_REACHABLE_APPROACH && corridor.selection() == null,
            "no real A* route must abstain from geometric selection");
        check(corridorBackend.results.size() == 1 && corridorBackend.results.get(0).status() == PlannerBackend.Status.NO_PATH,
            "real A* reports NO_PATH for a closed conservative corridor");
    }

    private static void cancellationAndPureGeometry() {
        var clock = new AtomicLong(NOW_US);
        var world = snapshot(List.of(track(1, "piece", 6, 2)));
        var originalFacts = facts(world);
        var backend = new RecordingBackend(clock);
        var cancelled = selector(backend, clock).select(request("already-cancelled", TargetSelector.Mode.SELECTED_OBJECT,
            1L, world, List.of(), () -> true));
        check(cancelled.status() == TargetSelector.Status.CANCELLED && cancelled.selection() == null && backend.requests.isEmpty(),
            "pre-cancelled selector work cannot invoke the backend");
        var flag = new AtomicBoolean();
        var duringSearchBackend = new RecordingBackend(clock);
        PlannerBackend cancelBeforeSearch = plan -> { flag.set(true); return duringSearchBackend.plan(plan); };
        var inSearch = selector(cancelBeforeSearch, clock).select(request("cancelled-at-backend", TargetSelector.Mode.SELECTED_OBJECT,
            1L, world, List.of(), flag::get));
        check(duringSearchBackend.results.size() == 1
            && duringSearchBackend.results.get(0).status() == PlannerBackend.Status.CANCELLED,
            "real A* observes cooperative cancellation at the backend boundary");
        check(inSearch.status() == TargetSelector.Status.CANCELLED && inSearch.selection() == null,
            "cancelled real backend reply cannot leave a selected goal");
        var geometryBackend = new RecordingBackend(clock);
        selected(selector(geometryBackend, clock).select(request("geometry-only", TargetSelector.Mode.SELECTED_OBJECT,
            1L, world, List.of(), () -> false)), "geometric reachability is available before possession");
        check(!originalFacts.possessionVerified() && originalFacts.equals(facts(world)),
            "selector must neither invent possession nor modify authoritative robot facts");
        check(geometryBackend.results.stream().allMatch(r -> r.trajectory() == null && r.path() != null),
            "selector proof stays geometric and contains no timed follower execution");
        check(world.id() == 1 && world.tracks().get(0).lastMeasurementUs() == NOW_US,
            "selection does not mutate the immutable tracking snapshot");
    }
}
