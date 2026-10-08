package org.frcworldstate.core;

import java.util.List;
import java.util.Set;
import static org.frcworldstate.core.Geometry.*;

/** Dependency-free synthetic contract checks. Values here are not hardware safety qualifications. */
public final class PickupSafetyTest {
    private PickupSafetyTest() {}
    private static final Uncertainty UNCERTAINTY = new Uncertainty(0.0001, 0, 0.0001);
    private static final Pose2 START = pose(1, 1);
    private static final PlannerBackend.Bounds BOUNDS = new PlannerBackend.Bounds(0, 0, 20, 20);
    private static final PlannerBackend.Footprint FOOTPRINT = new PlannerBackend.Footprint(0.4, 0.4);
    private static final FieldIdentity FIELD = new FieldIdentity("offseason", "synthetic", "1");

    public static void run() {
        asynchronousReplyAndMeasuredCompletion();
        missingPossessionRecoveryIsBounded();
        deterministicReachableSelectionAndLock();
        hysteresisOnRecovery();
        replanThrottlingAndOldReplies();
        movedStartAndMalformedEndpoints();
        taskReplacementAndEpochCancellation();
        freshRelativeAlignmentRequired();
        independentCollisionWhileFollowing();
        supervisorReasons();
        pathExpiryAndFutureFeedback();
        plannerFailureAndTimeoutContracts();
    }

    private static void asynchronousReplyAndMeasuredCompletion() {
        Fixture f = new Fixture();
        PickupController.Decision issued = f.step(100_000, START);
        PlannerBackend.Request request = issued.plannerRequest();
        check(request != null && issued.state() == PickupController.State.PLANNING, "planner request missing");
        check(!issued.completionEvent(), "planning cannot complete task");
        PickupController.Decision accepted = f.step(110_000, START, Velocity2.zero(), f.track(110_000), null, success(request));
        check(accepted.snapshotId() != request.snapshotId(), "test must advance snapshot ID");
        check(accepted.state() == PickupController.State.TRANSIT && accepted.path() != null, "reply must bind original request snapshot");
        Pose2 goal = request.goal().pose();
        check(f.step(120_000, goal, new Velocity2(new Vec2(0.2, 0), 0), f.track(120_000), null, null).state() == PickupController.State.TRANSIT,
                "arrival with excess measured velocity must not advance");
        f.step(130_000, goal);
        check(f.step(150_000, goal).state() == PickupController.State.APPROACH, "settled arrival must require relative approach");
        check(f.step(160_000, goal, Velocity2.zero(), f.track(160_000), alignment(160_000), null).state() == PickupController.State.ACQUIRE, "fresh alignment missing");
        check(!f.step(170_000, goal, Velocity2.zero(), f.track(170_000), alignment(170_000), null).completionEvent(), "duration cannot prove possession");
        f.possession = true;
        check(f.step(180_000, goal, Velocity2.zero(), List.of(), null, null).state() == PickupController.State.VERIFY,
                "measured possession may enter verification after target disappears");
        f.possession = false;
        check(!f.step(190_000, goal, Velocity2.zero(), List.of(), null, null).completionEvent(), "missing possession cannot complete verification");
        f.possession = true;
        f.mechanismReady = false;
        check(!f.step(200_000, goal, Velocity2.zero(), List.of(), null, null).completionEvent(), "mechanism evidence required");
        f.mechanismReady = true;
        f.step(210_000, goal, Velocity2.zero(), List.of(), null, null);
        PickupController.Decision done = f.step(230_000, goal, Velocity2.zero(), List.of(), null, null);
        check(done.state() == PickupController.State.SUCCEEDED && done.completionEvent(), "measured settled possession should complete once");
        check(!f.step(240_000, goal, Velocity2.zero(), List.of(), null, null).completionEvent(), "duplicate completion event");
    }

    private static void missingPossessionRecoveryIsBounded() {
        Fixture f = new Fixture();
        PlannerBackend.Request request = f.toAcquire();
        PickupController.Decision failure = f.step(310_000, request.goal().pose(), Velocity2.zero(), f.track(310_000), alignment(310_000), null);
        check(failure.state() == PickupController.State.RECOVER && !failure.completionEvent(), "possession timeout must recover");
        check(f.step(330_000, START).state() == PickupController.State.SELECT, "bounded recovery wait");
        PlannerBackend.Request retry = f.step(340_000, START).plannerRequest();
        PickupController.Decision exhausted = f.step(350_000, START, Velocity2.zero(), f.track(350_000), null, result(retry, PlannerBackend.Status.NO_PATH, null));
        check(exhausted.state() == PickupController.State.FAILED && exhausted.attempts() == 2 && exhausted.recoveries() == 1, "retry limits must fail deterministically");
    }

    private static void deterministicReachableSelectionAndLock() {
        Fixture f = new Fixture();
        List<World.ObjectTrack> ties = List.of(track(2, 100_000, new Vec2(3, 1)), track(1, 100_000, new Vec2(3, 1)));
        PickupController.Decision selected = f.step(100_000, START, Velocity2.zero(), ties, null, null);
        check(selected.targetId() == 1, "deterministic tie must use track identity");
        PlannerBackend.Request request = selected.plannerRequest();
        List<World.ObjectTrack> closer = List.of(track(1, 110_000, new Vec2(3, 1)), track(0, 110_000, new Vec2(1.8, 1)));
        check(f.step(110_000, START, Velocity2.zero(), closer, null, success(request)).targetId() == 1, "active target must remain locked");
        Fixture filtered = new Fixture(PickupController.distanceScorer((target, snapshot) -> target.id() != 1));
        check(filtered.step(100_000, START, Velocity2.zero(), ties, null, null).targetId() == 2, "unreachable candidates must be excluded before distance scoring");
    }

    private static void hysteresisOnRecovery() {
        Fixture f = new Fixture();
        PlannerBackend.Request request = f.step(100_000, START).plannerRequest();
        f.step(110_000, START, Velocity2.zero(), f.track(110_000), null, result(request, PlannerBackend.Status.TIMEOUT, null));
        f.step(130_000, START);
        List<World.ObjectTrack> almostBetter = List.of(track(1, 150_000, new Vec2(3, 1)), track(2, 150_000, new Vec2(2.8, 1)));
        check(f.step(150_000, START, Velocity2.zero(), almostBetter, null, null).targetId() == 1, "small cost changes must respect selection hysteresis");
    }

    private static void replanThrottlingAndOldReplies() {
        Fixture f = new Fixture();
        PlannerBackend.Request old = f.step(100_000, START).plannerRequest();
        f.mapVersion = 2;
        PickupController.Decision hold = f.step(120_000, START);
        check(old.cancellation().cancelled() && hold.plannerRequest() == null && hold.intent() == PickupController.Intent.HOLD, "changed map must cancel and throttle replans");
        PlannerBackend.Request fresh = f.step(150_000, START).plannerRequest();
        check(fresh != null && !fresh.requestId().equals(old.requestId()), "fresh request identity required");
        check(f.step(160_000, START, Velocity2.zero(), f.track(160_000), null, success(old)).state() == PickupController.State.PLANNING, "old planner reply must be rejected");
        check(f.step(170_000, START, Velocity2.zero(), f.track(170_000), null, success(fresh)).state() == PickupController.State.TRANSIT, "matching current map result must be accepted");
        List<World.ObjectTrack> moved = List.of(track(1, 180_000, new Vec2(3.4, 1)));
        PickupController.Decision movementHold = f.step(180_000, START, Velocity2.zero(), moved, null, null);
        check(movementHold.path() == null && movementHold.intent() == PickupController.Intent.HOLD, "meaningful target motion must stop following while replan is throttled");
    }

    private static void movedStartAndMalformedEndpoints() {
        Fixture moved = new Fixture();
        PlannerBackend.Request request = moved.step(100_000, START).plannerRequest();
        PickupController.Decision staleStart = moved.step(160_000, pose(1.4, 1), Velocity2.zero(), moved.track(160_000), null, success(request));
        check(staleStart.state() == PickupController.State.PLANNING && staleStart.plannerRequest() != null, "moved start must replan before accepting geometry");
        Fixture bad = new Fixture();
        PlannerBackend.Request r = bad.step(100_000, START).plannerRequest();
        PlannerBackend.GeometricPath malformed = new PlannerBackend.GeometricPath(List.of(pose(5, 5), r.goal().pose()));
        check(bad.step(110_000, START, Velocity2.zero(), bad.track(110_000), null, result(r, PlannerBackend.Status.SUCCESS, malformed)).state() == PickupController.State.RECOVER,
                "path start must match request start");
        Fixture wrongEnd = new Fixture();
        PlannerBackend.Request endRequest = wrongEnd.step(100_000, START).plannerRequest();
        PlannerBackend.GeometricPath wrongGoal = new PlannerBackend.GeometricPath(List.of(START, pose(5, 5)));
        check(wrongEnd.step(110_000, START, Velocity2.zero(), wrongEnd.track(110_000), null, result(endRequest, PlannerBackend.Status.SUCCESS, wrongGoal)).state() == PickupController.State.RECOVER,
                "path endpoint must match goal tolerances");
    }

    private static void taskReplacementAndEpochCancellation() {
        Fixture f = new Fixture();
        PlannerBackend.Request old = f.step(100_000, START).plannerRequest();
        f.controller.start("replacement", 1, 110_000);
        PlannerBackend.Request current = f.step(110_000, START).plannerRequest();
        check(old.cancellation().cancelled() && current.taskId().equals("replacement"), "task replacement must cancel old work");
        check(f.step(120_000, START, Velocity2.zero(), f.track(120_000), null, success(old)).state() == PickupController.State.PLANNING, "old task reply cannot complete replacement");
        f.epoch = 2;
        PickupController.Decision reset = f.step(130_000, START);
        check(reset.state() == PickupController.State.CANCELLED && reset.safety().reason() == SafetySupervisor.Reason.EPOCH_CHANGED && current.cancellation().cancelled(), "epoch reset must cancel goals and work");
        Fixture cancelled = new Fixture();
        PlannerBackend.Request pending = cancelled.step(100_000, START).plannerRequest();
        cancelled.controller.cancel(110_000);
        check(pending.cancellation().cancelled() && cancelled.step(110_000, START).state() == PickupController.State.CANCELLED, "explicit cancellation");
    }

    private static void freshRelativeAlignmentRequired() {
        Fixture f = new Fixture();
        PlannerBackend.Request request = f.toApproach();
        Pose2 goal = request.goal().pose();
        check(f.step(160_000, goal).intent() == PickupController.Intent.HOLD, "field arrival alone cannot acquire");
        PickupController.Alignment stale = new PickupController.Alignment(1, 1, 100_000, new Vec2(.5, 0), UNCERTAINTY);
        check(f.step(170_000, goal, Velocity2.zero(), f.track(170_000), stale, null).intent() == PickupController.Intent.HOLD, "stale alignment rejected");
        PickupController.Alignment wrong = new PickupController.Alignment(2, 1, 180_000, new Vec2(.5, 0), UNCERTAINTY);
        check(f.step(180_000, goal, Velocity2.zero(), f.track(180_000), wrong, null).intent() == PickupController.Intent.HOLD, "alignment identity switch rejected");
        PickupController.Alignment future = new PickupController.Alignment(1, 1, 200_001, new Vec2(.5, 0), UNCERTAINTY);
        check(f.step(190_000, goal, Velocity2.zero(), f.track(190_000), future, null).intent() == PickupController.Intent.HOLD, "future alignment rejected");
        PickupController.Alignment uncertain = new PickupController.Alignment(1, 1, 200_000, new Vec2(.5, 0), new Uncertainty(1, 0, 1));
        check(f.step(200_000, goal, Velocity2.zero(), f.track(200_000), uncertain, null).intent() == PickupController.Intent.HOLD, "uncertain alignment rejected");
        PickupController.Alignment offCenter = new PickupController.Alignment(1, 1, 210_000, new Vec2(.8, .2), UNCERTAINTY);
        check(f.step(210_000, goal, Velocity2.zero(), f.track(210_000), offCenter, null).intent() == PickupController.Intent.ALIGN_RELATIVE, "fresh offset must request relative alignment");
    }

    private static void independentCollisionWhileFollowing() {
        Fixture f = new Fixture();
        PlannerBackend.Request request = f.step(100_000, START).plannerRequest();
        f.step(110_000, START, Velocity2.zero(), f.track(110_000), null, success(request));
        f.obstacles = List.of(new PlannerBackend.Obstacle("approaching", new Vec2(1.8, 1), .2, 0, 5_000_000, true));
        PickupController.Decision stop = f.step(120_000, START, new Velocity2(new Vec2(1, 0), 0), f.track(120_000), null, null);
        check(stop.state() == PickupController.State.SAFE_STOP && stop.path() == null && stop.safety().reason() == SafetySupervisor.Reason.COLLISION_RISK,
                "following must independently stop for swept stopping-envelope collision");
        check(stop.safety().stoppingDistanceM() > .5, "collision test must include braking travel");
        f.obstacles = List.of();
        check(f.step(130_000, START).state() == PickupController.State.SAFE_STOP, "safe stop must latch");
    }

    private static void supervisorReasons() {
        Fixture disabled = new Fixture(); disabled.enabled = false; reason(disabled, SafetySupervisor.Reason.DISABLED, PickupController.State.SAFE_STOP);
        Fixture ds = new Fixture(); ds.dsConnected = false; reason(ds, SafetySupervisor.Reason.LOST_DS, PickupController.State.SAFE_STOP);
        Fixture cancel = new Fixture(); cancel.driverCancel = true; reason(cancel, SafetySupervisor.Reason.DRIVER_CANCEL, PickupController.State.CANCELLED);
        Fixture authority = new Fixture(); authority.authority = false; reason(authority, SafetySupervisor.Reason.AUTHORITY_CONFLICT, PickupController.State.SAFE_STOP);
        Fixture invalid = new Fixture(); invalid.localizationValid = false; reason(invalid, SafetySupervisor.Reason.INVALID_LOCALIZATION, PickupController.State.SAFE_STOP);
        Fixture sensing = new Fixture(); sensing.sensingFresh = false; reason(sensing, SafetySupervisor.Reason.STALE_REQUIRED_SENSING, PickupController.State.SAFE_STOP);
        Fixture collision = new Fixture(); collision.collisionFresh = false; reason(collision, SafetySupervisor.Reason.COLLISION_INFORMATION_STALE, PickupController.State.SAFE_STOP);
        Fixture overrun = new Fixture(); overrun.loopDurationUs = 25_001; reason(overrun, SafetySupervisor.Reason.LOOP_OVERRUN, PickupController.State.SAFE_STOP);
        Fixture mode = new Fixture(); mode.mode = "unsupported"; reason(mode, SafetySupervisor.Reason.UNSUPPORTED_MODE, PickupController.State.SAFE_STOP);
        Fixture expired = new Fixture(); expired.obstacles = List.of(new PlannerBackend.Obstacle("old", new Vec2(10, 10), .1, 0, 100_001, true));
        reason(expired, SafetySupervisor.Reason.COLLISION_INFORMATION_STALE, PickupController.State.SAFE_STOP);
        Fixture boundary = new Fixture();
        check(boundary.step(100_000, pose(.1, 1)).safety().reason() == SafetySupervisor.Reason.COLLISION_RISK, "footprint must remain within field bounds");
    }

    private static void pathExpiryAndFutureFeedback() {
        Fixture f = new Fixture();
        PlannerBackend.Request request = f.step(100_000, START).plannerRequest();
        PlannerBackend.Result shortPath = new PlannerBackend.Result(request.requestId(), request.taskId(), request.epoch(), request.snapshotId(), request.obstacleMapVersion(),
                PlannerBackend.Status.SUCCESS, new PlannerBackend.GeometricPath(List.of(request.startPose(), request.goal().pose())), null, 1_000, 130_000, "fake", "short validity");
        f.step(110_000, START, Velocity2.zero(), f.track(110_000), null, shortPath);
        check(f.step(130_000, START).path() == null, "expired geometry cannot be followed");
        Fixture future = new Fixture(); future.egoTimestampOffset = 1;
        reason(future, SafetySupervisor.Reason.INVALID_LOCALIZATION, PickupController.State.SAFE_STOP);
        Fixture older = new Fixture(); older.sensingTimestampOffset = -100_001;
        check(older.step(200_000, START).safety().reason() == SafetySupervisor.Reason.STALE_REQUIRED_SENSING, "stale measured feedback timestamp rejected");
    }

    private static void plannerFailureAndTimeoutContracts() {
        for (PlannerBackend.Status status : List.of(PlannerBackend.Status.NO_PATH, PlannerBackend.Status.TIMEOUT,
                PlannerBackend.Status.CANCELLED, PlannerBackend.Status.INVALID_INPUT, PlannerBackend.Status.STALE_RESULT)) {
            Fixture f = new Fixture();
            PlannerBackend.Request request = f.step(100_000, START).plannerRequest();
            PickupController.Decision failed = f.step(110_000, START, Velocity2.zero(), f.track(110_000), null, result(request, status, null));
            check(failed.state() == PickupController.State.RECOVER && !failed.completionEvent() && request.cancellation().cancelled(), "typed planner failure cannot finish pickup: " + status);
        }
        Fixture timeout = new Fixture();
        PlannerBackend.Request slow = timeout.step(100_000, START).plannerRequest();
        check(timeout.step(slow.validUntilUs(), START).state() == PickupController.State.RECOVER && slow.cancellation().cancelled(), "absent planner reply must time out and cancel");
        Fixture mismatch = new Fixture();
        PlannerBackend.Request request = mismatch.step(100_000, START).plannerRequest();
        PlannerBackend.Result wrongSnapshot = new PlannerBackend.Result(request.requestId(), request.taskId(), request.epoch(), request.snapshotId() + 1,
                request.obstacleMapVersion(), PlannerBackend.Status.SUCCESS, new PlannerBackend.GeometricPath(List.of(request.startPose(), request.goal().pose())),
                null, 1_000, request.validUntilUs(), "fake", "wrong snapshot");
        check(mismatch.step(110_000, START, Velocity2.zero(), mismatch.track(110_000), null, wrongSnapshot).state() == PickupController.State.PLANNING,
                "reply request snapshot mismatch rejected");
        PlannerBackend.Result overBudget = new PlannerBackend.Result(request.requestId(), request.taskId(), request.epoch(), request.snapshotId(),
                request.obstacleMapVersion(), PlannerBackend.Status.SUCCESS, new PlannerBackend.GeometricPath(List.of(request.startPose(), request.goal().pose())),
                null, request.budget().limitNanos() + 1, request.validUntilUs(), "fake", "over budget");
        check(mismatch.step(120_000, START, Velocity2.zero(), mismatch.track(120_000), null, overBudget).state() == PickupController.State.RECOVER,
                "over-budget solver reply rejected");
        Fixture disappeared = new Fixture();
        PlannerBackend.Request active = disappeared.step(100_000, START).plannerRequest();
        PickupController.Decision empty = disappeared.step(110_000, START, Velocity2.zero(), List.of(), null, null);
        check(empty.state() == PickupController.State.RECOVER && !empty.completionEvent() && active.cancellation().cancelled(), "target disappearance is not completion evidence");
    }

    private static void reason(Fixture fixture, SafetySupervisor.Reason reason, PickupController.State state) {
        PickupController.Decision decision = fixture.step(100_000, START);
        check(decision.safety().reason() == reason && decision.state() == state && !decision.completionEvent(), "wrong safety outcome: " + reason + "/" + decision.safety().reason());
    }
    private static PickupController.Alignment alignment(long nowUs) { return new PickupController.Alignment(1, 1, nowUs, new Vec2(.5, 0), UNCERTAINTY); }
    private static Pose2 pose(double x, double y) { return new Pose2(new Vec2(x, y), 0); }
    private static PlannerBackend.Result success(PlannerBackend.Request request) {
        return result(request, PlannerBackend.Status.SUCCESS, new PlannerBackend.GeometricPath(List.of(request.startPose(), request.goal().pose())));
    }
    private static PlannerBackend.Result result(PlannerBackend.Request request, PlannerBackend.Status status, PlannerBackend.GeometricPath path) {
        return new PlannerBackend.Result(request.requestId(), request.taskId(), request.epoch(), request.snapshotId(), request.obstacleMapVersion(), status, path, null,
                1_000, request.validUntilUs(), "fake", "synthetic test");
    }
    private static World.ObjectTrack track(long id, long nowUs, Vec2 position) {
        return new World.ObjectTrack(id, 1, "piece", position, new Vec2(0, 0), nowUs, nowUs, .95, UNCERTAINTY,
                World.TrackLifecycle.CONFIRMED, World.EstimateKind.MEASURED, List.of());
    }
    private static void check(boolean condition, String message) { if (!condition) throw new AssertionError(message); }

    private static final class Fixture {
        private final PickupController controller;
        private long snapshotId, epoch = 1, mapVersion = 1, loopDurationUs = 1_000, egoTimestampOffset, sensingTimestampOffset;
        private boolean enabled = true, dsConnected = true, driverCancel, authority = true, mechanismReady = true, possession, sensingFresh = true, collisionFresh = true, localizationValid = true;
        private String mode = "AUTO";
        private List<PlannerBackend.Obstacle> obstacles = List.of();
        Fixture() { this(PickupController.distanceScorer((target, snapshot) -> true)); }
        Fixture(PickupController.ReachableScorer scorer) {
            SafetySupervisor safety = new SafetySupervisor(new SafetySupervisor.Config(100_000, 100_000, 25_000,
                    1, .1, .05, 2, FOOTPRINT, true, SafetySupervisor.Action.HOLD, Set.of("AUTO")));
            PickupController.Motion motion = new PickupController.Motion(new Vec2(.5, 0), .05, .1, .05, .1, 20_000, 30_000, .05, .1, .01);
            PickupController.Planning planning = new PickupController.Planning(FOOTPRINT, new PlannerBackend.Constraints(2, 2, 2), BOUNDS, 10_000_000, .15, .1);
            PickupController.Config config = new PickupController.Config("piece", .7, 500_000, 100_000, 2_000_000, 50_000,
                    .2, 5_000_000, 200_000, 150_000, 150_000, 20_000, 2, 1, .3, motion, planning);
            controller = new PickupController(config, safety, scorer); controller.start("task", 1, 100_000);
        }
        List<World.ObjectTrack> track(long nowUs) {
            if (epoch != 1) return List.of();
            return List.of(PickupSafetyTest.track(1, nowUs, new Vec2(3, 1)));
        }
        PickupController.Decision step(long nowUs, Pose2 pose) { return step(nowUs, pose, Velocity2.zero(), track(nowUs), null, null); }
        PickupController.Decision step(long nowUs, Pose2 pose, Velocity2 velocity, List<World.ObjectTrack> tracks, PickupController.Alignment alignment, PlannerBackend.Result reply) {
            World.EgoState ego = new World.EgoState(nowUs + egoTimestampOffset, pose, velocity, UNCERTAINTY, localizationValid);
            World.WorldSnapshot snapshot = new World.WorldSnapshot(++snapshotId, epoch, mapVersion, FIELD, ego, tracks);
            World.RobotFacts facts = new World.RobotFacts(ego, enabled, dsConnected, mode, driverCancel, authority, mechanismReady, possession, sensingFresh, Math.max(0, nowUs + sensingTimestampOffset));
            return controller.update(snapshot, facts, new PickupController.Update(nowUs, nowUs * 1_000, loopDurationUs, obstacles, collisionFresh, alignment, reply));
        }
        PlannerBackend.Request toApproach() {
            PlannerBackend.Request request = step(100_000, START).plannerRequest();
            step(110_000, START, Velocity2.zero(), track(110_000), null, success(request));
            step(130_000, request.goal().pose());
            check(step(150_000, request.goal().pose()).state() == PickupController.State.APPROACH, "fixture failed to approach");
            return request;
        }
        PlannerBackend.Request toAcquire() {
            PlannerBackend.Request request = toApproach();
            check(step(160_000, request.goal().pose(), Velocity2.zero(), track(160_000), alignment(160_000), null).state() == PickupController.State.ACQUIRE, "fixture failed to acquire");
            return request;
        }
    }
}
