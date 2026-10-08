package org.frcworldstate.core;

import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import static org.frcworldstate.core.Geometry.*;

/**
 * Deterministic pickup task state machine. Robot integration executes the returned intents
 * and supplies authoritative measured feedback. No planner, predictor, I/O or motors run here.
 * A safe stop latches until a new task is explicitly started.
 */
public final class PickupController {
    public enum State { IDLE, SELECT, PLANNING, TRANSIT, APPROACH, ACQUIRE, VERIFY, RECOVER, SUCCEEDED, FAILED, CANCELLED, SAFE_STOP }
    public enum Intent { HOLD, REQUEST_PLAN, FOLLOW_GEOMETRIC_PATH, ALIGN_RELATIVE, REQUEST_ACQUIRE, VERIFY_POSSESSION, RECOVER, SAFE_STOP }
    public record Motion(Vec2 intakePickupPointM, double positionToleranceM, double headingToleranceRad,
            double velocityToleranceMps, double angularVelocityToleranceRadPerSec, long settlingUs,
            long maxAlignmentAgeUs, double alignmentToleranceM, double alignmentHeadingToleranceRad,
            double maxAlignmentVarianceM2) {
        public Motion {
            Objects.requireNonNull(intakePickupPointM);
            if (intakePickupPointM.norm() == 0) throw new IllegalArgumentException("intake pickup point must define a direction");
            nonnegative(positionToleranceM); nonnegative(headingToleranceRad); nonnegative(velocityToleranceMps);
            nonnegative(angularVelocityToleranceRadPerSec); time(settlingUs); SafetySupervisor.positive(maxAlignmentAgeUs);
            nonnegative(alignmentToleranceM); nonnegative(alignmentHeadingToleranceRad);
            nonnegative(maxAlignmentVarianceM2);
        }
    }
    public record Planning(PlannerBackend.Footprint footprint, PlannerBackend.Constraints constraints,
            PlannerBackend.Bounds bounds, long searchBudgetNanos, double maxResultStartDisplacementM,
            double maxResultStartHeadingDriftRad) {
        public Planning { Objects.requireNonNull(footprint); Objects.requireNonNull(constraints); Objects.requireNonNull(bounds); SafetySupervisor.positive(searchBudgetNanos); nonnegative(maxResultStartDisplacementM); nonnegative(maxResultStartHeadingDriftRad); }
    }
    /** Every duration/margin is robot/season configuration. Synthetic tests do not qualify it. */
    public record Config(String objectClass, double minimumConfidence, long trackFreshUs,
            long selectionTimeoutUs, long requestLifetimeUs, long minimumReplanIntervalUs,
            double meaningfulTargetMovementM, long attemptTimeoutUs, long approachTimeoutUs,
            long acquisitionTimeoutUs, long verificationTimeoutUs, long recoveryWaitUs,
            int maxAttempts, int maxRecoveries, double selectionHysteresisCost, Motion motion, Planning planning) {
        public Config {
            id(objectClass); nonnegative(minimumConfidence); if (minimumConfidence > 1) throw new IllegalArgumentException("confidence > 1");
            SafetySupervisor.positive(trackFreshUs); SafetySupervisor.positive(selectionTimeoutUs); SafetySupervisor.positive(requestLifetimeUs);
            time(minimumReplanIntervalUs); nonnegative(meaningfulTargetMovementM); SafetySupervisor.positive(attemptTimeoutUs);
            SafetySupervisor.positive(approachTimeoutUs); SafetySupervisor.positive(acquisitionTimeoutUs); SafetySupervisor.positive(verificationTimeoutUs);
            time(recoveryWaitUs); if (maxAttempts <= 0 || maxRecoveries < 0) throw new IllegalArgumentException("invalid retry bounds");
            nonnegative(selectionHysteresisCost); Objects.requireNonNull(motion); Objects.requireNonNull(planning);
        }
    }
    /** Smaller finite cost wins. Reachability is explicit and must come from a bounded deterministic check. */
    public record CandidateScore(boolean reachable, double cost) { public CandidateScore { finite(cost); } }
    @FunctionalInterface public interface ReachableScorer { CandidateScore score(World.ObjectTrack target, World.WorldSnapshot snapshot); }
    /** Fresh accepted robot-relative measurement; never reconstructed from a stale field target. */
    public record Alignment(long targetId, long epoch, long observationUs, Vec2 robotRelativeM, Uncertainty uncertainty) {
        public Alignment { time(targetId); time(epoch); time(observationUs); Objects.requireNonNull(robotRelativeM); Objects.requireNonNull(uncertainty); }
    }
    public record Update(long nowUs, long monotonicNanos, long loopDurationUs,
            List<PlannerBackend.Obstacle> obstacles, boolean collisionInformationFresh,
            Alignment alignment, PlannerBackend.Result plannerReply) {
        public Update { time(nowUs); time(loopDurationUs); obstacles = List.copyOf(obstacles); if (obstacles.size() > 4096) throw new IllegalArgumentException("obstacle capacity exceeded"); }
    }
    /** Null request/path/alignment values mean that intent carries no such payload. */
    public record Decision(long id, String taskId, long epoch, long snapshotId, long timeUs,
            State state, Intent intent, Long targetId, PlannerBackend.Request plannerRequest,
            PlannerBackend.GeometricPath path, Vec2 alignmentErrorM, double alignmentHeadingErrorRad,
            boolean completionEvent, int attempts, int recoveries, SafetySupervisor.Decision safety, String detail) {
        public Decision {
            time(id); Geometry.id(taskId); time(epoch); time(snapshotId); time(timeUs); Objects.requireNonNull(state); Objects.requireNonNull(intent);
            finite(alignmentHeadingErrorRad); Objects.requireNonNull(safety); Objects.requireNonNull(detail);
        }
    }

    private final Config config;
    private final SafetySupervisor supervisor;
    private final ReachableScorer scorer;
    private final Set<Long> unreachableTargets = new HashSet<>();
    private State state = State.IDLE;
    private String taskId = "idle";
    private long epoch, nextDecisionId, requestSequence, stateSinceUs, attemptSinceUs, lastUpdateUs = -1;
    private long lastPlanUs = -1, settledSinceUs = -1;
    private Long targetId, preferredTargetId;
    private World.ObjectTrack lockedTarget;
    private World.WorldSnapshot requestSnapshot;
    private PlannerBackend.Request pendingRequest;
    private AtomicBoolean pendingCancellation;
    private PlannerBackend.GeometricPath path;
    private PlannerBackend.Goal goal;
    private Vec2 plannedTargetPosition;
    private long plannedMapVersion;
    private long pathValidUntilUs;
    private int attempts, recoveries;
    private boolean completionReported;
    private String detail = "idle";

    public PickupController(Config config, SafetySupervisor supervisor, ReachableScorer scorer) {
        this.config = Objects.requireNonNull(config); this.supervisor = Objects.requireNonNull(supervisor); this.scorer = Objects.requireNonNull(scorer);
        if (!config.planning().footprint().equals(supervisor.config().footprint())) throw new IllegalArgumentException("planning/safety footprint mismatch");
    }
    /** A supplied reachability filter is required; distance only orders its reachable candidates. */
    public static ReachableScorer distanceScorer(java.util.function.BiPredicate<World.ObjectTrack, World.WorldSnapshot> reachable) {
        Objects.requireNonNull(reachable);
        return (target, snapshot) -> new CandidateScore(reachable.test(target, snapshot), target.positionM().subtract(snapshot.ego().fieldPose().position()).norm());
    }
    public State state() { return state; }
    public String taskId() { return taskId; }

    /** Replacing a task cancels its outstanding request. Globally increasing request IDs reject its late replies. */
    public void start(String newTaskId, long newEpoch, long nowUs) {
        id(newTaskId); time(newEpoch); time(nowUs);
        if (newTaskId.equals(taskId) && newEpoch == epoch && state != State.IDLE) throw new IllegalArgumentException("task identity must change on replacement/restart");
        if (lastUpdateUs >= 0 && nowUs < lastUpdateUs) throw new IllegalArgumentException("task start moved backwards");
        cancelPending(); taskId = newTaskId; epoch = newEpoch; state = State.SELECT; stateSinceUs = nowUs;
        attemptSinceUs = nowUs; lastPlanUs = -1; settledSinceUs = -1; targetId = null; preferredTargetId = null;
        lockedTarget = null; path = null; goal = null; plannedTargetPosition = null; unreachableTargets.clear();
        attempts = 0; recoveries = 0; completionReported = false; detail = "selecting target";
    }
    public void cancel(long nowUs) {
        time(nowUs); if (lastUpdateUs >= 0 && nowUs < lastUpdateUs) throw new IllegalArgumentException("cancellation moved backwards");
        if (terminal(state) || state == State.IDLE) return;
        cancelPending(); path = null; transition(State.CANCELLED, nowUs, "task cancelled");
    }

    public Decision update(World.WorldSnapshot snapshot, World.RobotFacts facts, Update update) {
        Objects.requireNonNull(snapshot); Objects.requireNonNull(facts); Objects.requireNonNull(update);
        if (snapshot.tracks().size() > 4096) throw new IllegalArgumentException("track capacity exceeded");
        if (lastUpdateUs >= 0 && update.nowUs() < lastUpdateUs) throw new IllegalArgumentException("nonmonotonic control update");
        if (update.nowUs() < stateSinceUs) throw new IllegalArgumentException("update before task start");
        lastUpdateUs = update.nowUs();
        SafetySupervisor.Decision safety = supervisor.evaluate(new SafetySupervisor.Input(state == State.IDLE ? snapshot.epoch() : epoch,
                snapshot, facts, update.nowUs(), update.loopDurationUs(), update.obstacles(), update.collisionInformationFresh(), config.planning().bounds()));
        if (!safety.permitsMotion()) {
            cancelPending(); path = null;
            if (!terminal(state)) transition(safety.reason() == SafetySupervisor.Reason.DRIVER_CANCEL || safety.reason() == SafetySupervisor.Reason.EPOCH_CHANGED
                    ? State.CANCELLED : State.SAFE_STOP, update.nowUs(), "safety: " + safety.reason());
            return decision(snapshot, update, safety, Intent.SAFE_STOP, null, null, 0, false);
        }
        if (terminal(state) || state == State.IDLE) return decision(snapshot, update, safety, state == State.SAFE_STOP ? Intent.SAFE_STOP : Intent.HOLD, null, null, 0, false);

        if (state != State.SELECT && state != State.RECOVER && update.nowUs() - attemptSinceUs >= config.attemptTimeoutUs())
            recover(update.nowUs(), "attempt timed out", false);
        World.ObjectTrack current = findTarget(snapshot, targetId, update.nowUs());
        if (targetId != null && state != State.RECOVER && state != State.VERIFY && current == null
                && !(state == State.ACQUIRE && facts.possessionVerified() && facts.mechanismReady())) recover(update.nowUs(), "target observation expired", false);
        else if (current != null) lockedTarget = current;

        PlannerBackend.Request issued = null;
        Vec2 alignmentError = null;
        double headingError = 0;
        Intent intent = Intent.HOLD;
        boolean completed = false;
        switch (state) {
            case SELECT -> {
                World.ObjectTrack selected = select(snapshot, update.nowUs());
                if (selected == null) {
                    if (update.nowUs() - stateSinceUs >= config.selectionTimeoutUs()) transition(State.FAILED, update.nowUs(), "no reachable fresh target");
                } else if (attempts >= config.maxAttempts()) transition(State.FAILED, update.nowUs(), "attempt limit reached");
                else {
                    attempts++; attemptSinceUs = update.nowUs(); lockedTarget = selected; targetId = selected.id(); preferredTargetId = targetId;
                    issued = issuePlan(snapshot, facts, update); intent = issued == null ? Intent.HOLD : Intent.REQUEST_PLAN;
                }
            }
            case PLANNING -> {
                if (meaningfulChange(snapshot, lockedTarget)) {
                    cancelPending(); path = null;
                    issued = issuePlan(snapshot, facts, update); intent = issued == null ? Intent.HOLD : Intent.REQUEST_PLAN;
                } else if (pendingRequest == null) {
                    issued = issuePlan(snapshot, facts, update); intent = issued == null ? Intent.HOLD : Intent.REQUEST_PLAN;
                } else if (update.nowUs() >= pendingRequest.validUntilUs()) recover(update.nowUs(), "planner request timed out", false);
                else if (update.plannerReply() != null) {
                    PlannerBackend.Result reply = update.plannerReply();
                    if (!reply.matches(pendingRequest, requestSnapshot, update.nowUs()) || snapshot.epoch() != pendingRequest.epoch()
                            || !snapshot.field().equals(pendingRequest.field())
                            || snapshot.obstacleMapVersion() != pendingRequest.obstacleMapVersion()) detail = "ignored stale or mismatched planner reply";
                    else if (reply.solverDurationNanos() > pendingRequest.budget().limitNanos()) recover(update.nowUs(), "planner exceeded monotonic search budget", false);
                    else if (!startValid(facts.ego().fieldPose(), pendingRequest.startPose())) {
                        cancelPending(); issued = issuePlan(snapshot, facts, update); intent = issued == null ? Intent.HOLD : Intent.REQUEST_PLAN;
                        detail = "request start pose moved; replanning";
                    } else if (reply.status() == PlannerBackend.Status.SUCCESS && pathEndpointsValid(reply.path(), pendingRequest)) {
                        path = reply.path(); pathValidUntilUs = reply.validUntilUs(); cancelPending(); transition(State.TRANSIT, update.nowUs(), "accepted geometric path; awaiting execution feedback"); intent = Intent.FOLLOW_GEOMETRIC_PATH;
                    } else recover(update.nowUs(), "planner outcome: " + reply.status(), reply.status() == PlannerBackend.Status.NO_PATH || reply.status() == PlannerBackend.Status.INVALID_INPUT);
                }
            }
            case TRANSIT -> {
                if (meaningfulChange(snapshot, lockedTarget) || update.nowUs() >= pathValidUntilUs) {
                    path = null; issued = issuePlan(snapshot, facts, update); intent = issued == null ? Intent.HOLD : Intent.REQUEST_PLAN;
                    if (issued == null) transition(State.PLANNING, update.nowUs(), "holding until replan interval");
                } else if (settled(facts.ego(), update.nowUs())) { transition(State.APPROACH, update.nowUs(), "arrived; fresh relative alignment required"); intent = Intent.HOLD; }
                else intent = Intent.FOLLOW_GEOMETRIC_PATH;
            }
            case APPROACH -> {
                if (update.nowUs() - stateSinceUs >= config.approachTimeoutUs()) recover(update.nowUs(), "alignment timed out", false);
                else if (freshAlignment(update)) {
                    alignmentError = update.alignment().robotRelativeM().subtract(config.motion().intakePickupPointM()); headingError = alignmentHeadingError(update.alignment());
                    if (aligned(alignmentError, headingError) && arrived(facts.ego()) && facts.mechanismReady()) {
                        transition(State.ACQUIRE, update.nowUs(), "aligned with measured ready mechanism"); intent = Intent.REQUEST_ACQUIRE;
                    } else intent = Intent.ALIGN_RELATIVE;
                } else detail = "holding for fresh target-relative alignment";
            }
            case ACQUIRE -> {
                if (update.nowUs() - stateSinceUs >= config.acquisitionTimeoutUs()) recover(update.nowUs(), "acquisition timed out without measured possession", false);
                else if (facts.possessionVerified() && facts.mechanismReady()) {
                    transition(State.VERIFY, update.nowUs(), "measured possession; verifying settled execution feedback"); settledSinceUs = -1; intent = Intent.VERIFY_POSSESSION;
                } else if (freshAlignment(update)) {
                    alignmentError = update.alignment().robotRelativeM().subtract(config.motion().intakePickupPointM()); headingError = alignmentHeadingError(update.alignment());
                    if (aligned(alignmentError, headingError) && arrived(facts.ego()) && facts.mechanismReady()) intent = Intent.REQUEST_ACQUIRE;
                    else { transition(State.APPROACH, update.nowUs(), "alignment changed during acquisition"); intent = Intent.ALIGN_RELATIVE; }
                } else detail = "holding acquisition for fresh alignment";
            }
            case VERIFY -> {
                intent = Intent.VERIFY_POSSESSION;
                if (!facts.possessionVerified() || !facts.mechanismReady()) settledSinceUs = -1;
                else if (settled(facts.ego(), update.nowUs())) {
                    transition(State.SUCCEEDED, update.nowUs(), "measured settled arrival and verified mechanism/possession"); path = null;
                    if (!completionReported) { completionReported = true; completed = true; } intent = Intent.HOLD;
                }
                if (state == State.VERIFY && update.nowUs() - stateSinceUs >= config.verificationTimeoutUs()) recover(update.nowUs(), "possession verification timed out", false);
            }
            case RECOVER -> {
                intent = Intent.RECOVER;
                if (update.nowUs() - stateSinceUs >= config.recoveryWaitUs()) {
                    targetId = null; lockedTarget = null; goal = null;
                    transition(State.SELECT, update.nowUs(), "recovery complete; deterministic reselection"); intent = Intent.HOLD;
                }
            }
            default -> { }
        }
        if (state == State.FAILED || state == State.RECOVER) intent = state == State.RECOVER ? Intent.RECOVER : Intent.HOLD;
        return decision(snapshot, update, safety, intent, issued, alignmentError, headingError, completed);
    }

    private World.ObjectTrack select(World.WorldSnapshot snapshot, long nowUs) {
        World.ObjectTrack best = null, preferred = null;
        double bestCost = Double.POSITIVE_INFINITY, preferredCost = Double.POSITIVE_INFINITY;
        for (World.ObjectTrack track : snapshot.tracks()) {
            if (!eligible(track, nowUs) || unreachableTargets.contains(track.id())) continue;
            CandidateScore score = Objects.requireNonNull(scorer.score(track, snapshot));
            if (!score.reachable()) continue;
            if (preferredTargetId != null && preferredTargetId == track.id()) { preferred = track; preferredCost = score.cost(); }
            if (score.cost() < bestCost || (score.cost() == bestCost && (best == null || track.id() < best.id()))) { best = track; bestCost = score.cost(); }
        }
        return preferred != null && preferredCost <= bestCost + config.selectionHysteresisCost() ? preferred : best;
    }
    private boolean eligible(World.ObjectTrack target, long nowUs) {
        return target.epoch() == epoch && target.objectClass().equals(config.objectClass()) && target.confidence() >= config.minimumConfidence()
                && target.lifecycle() != World.TrackLifecycle.TENTATIVE && SafetySupervisor.fresh(target.lastMeasurementUs(), nowUs, config.trackFreshUs());
    }
    private World.ObjectTrack findTarget(World.WorldSnapshot snapshot, Long id, long nowUs) {
        if (id == null) return null;
        for (World.ObjectTrack track : snapshot.tracks()) if (track.id() == id && eligible(track, nowUs)) return track;
        return null;
    }
    private boolean meaningfulChange(World.WorldSnapshot snapshot, World.ObjectTrack target) {
        return target != null && plannedTargetPosition != null && (snapshot.obstacleMapVersion() != plannedMapVersion
                || target.positionM().subtract(plannedTargetPosition).norm() > config.meaningfulTargetMovementM());
    }
    private PlannerBackend.Request issuePlan(World.WorldSnapshot snapshot, World.RobotFacts facts, Update update) {
        if (lastPlanUs >= 0 && update.nowUs() - lastPlanUs < config.minimumReplanIntervalUs()) { detail = "holding until replan interval"; return null; }
        cancelPending(); path = null;
        double intakeAngle = Math.atan2(config.motion().intakePickupPointM().y(), config.motion().intakePickupPointM().x());
        Vec2 towardTarget = lockedTarget.positionM().subtract(facts.ego().fieldPose().position());
        double heading = towardTarget.norm() == 0 ? facts.ego().fieldPose().headingRad() : Math.atan2(towardTarget.y(), towardTarget.x()) - intakeAngle;
        Pose2 pose = new Pose2(lockedTarget.positionM().subtract(config.motion().intakePickupPointM().rotate(heading)), heading);
        goal = new PlannerBackend.Goal(pose, config.motion().positionToleranceM(), config.motion().headingToleranceRad(), config.motion().velocityToleranceMps());
        long validUntilUs = Math.addExact(update.nowUs(), config.requestLifetimeUs());
        for (PlannerBackend.Obstacle obstacle : update.obstacles()) {
            if (obstacle.validUntilUs() < validUntilUs) { recover(update.nowUs(), "collision envelope too short for planner request", false); return null; }
        }
        pendingCancellation = new AtomicBoolean();
        AtomicBoolean cancellation = pendingCancellation;
        pendingRequest = new PlannerBackend.Request(taskId + ":plan:" + ++requestSequence, taskId, epoch, snapshot.id(), snapshot.obstacleMapVersion(),
                snapshot.field(), facts.ego().fieldPose(), facts.ego().fieldVelocity(), goal, config.planning().footprint(), config.planning().constraints(),
                config.planning().bounds(), update.obstacles(), update.nowUs(), validUntilUs,
                new PlannerBackend.SearchBudget(update.monotonicNanos(), config.planning().searchBudgetNanos()), cancellation::get);
        requestSnapshot = snapshot; plannedTargetPosition = lockedTarget.positionM(); plannedMapVersion = snapshot.obstacleMapVersion(); lastPlanUs = update.nowUs();
        transition(State.PLANNING, update.nowUs(), "bounded planner request issued"); return pendingRequest;
    }
    private void cancelPending() {
        if (pendingCancellation != null) pendingCancellation.set(true);
        pendingCancellation = null; pendingRequest = null; requestSnapshot = null;
    }
    private void recover(long nowUs, String why, boolean unreachable) {
        cancelPending(); path = null; settledSinceUs = -1;
        if (unreachable && targetId != null) {
            // Bound per-task memory independently of snapshot track count.
            if (unreachableTargets.size() < 4096) unreachableTargets.add(targetId);
            preferredTargetId = null;
        }
        if (attempts >= config.maxAttempts() || recoveries >= config.maxRecoveries()) transition(State.FAILED, nowUs, why + "; retry limit reached");
        else { recoveries++; transition(State.RECOVER, nowUs, why); }
    }
    private boolean arrived(World.EgoState ego) {
        return goal != null && ego.fieldPose().position().subtract(goal.pose().position()).norm() <= goal.positionToleranceM()
                && Math.abs(angle(ego.fieldPose().headingRad() - goal.pose().headingRad())) <= goal.headingToleranceRad()
                && ego.fieldVelocity().linearMps().norm() <= goal.velocityToleranceMps()
                && Math.abs(ego.fieldVelocity().angularRadPerSec()) <= config.motion().angularVelocityToleranceRadPerSec();
    }
    private boolean settled(World.EgoState ego, long nowUs) {
        if (!arrived(ego)) { settledSinceUs = -1; return false; }
        if (settledSinceUs < 0) settledSinceUs = nowUs;
        return nowUs - settledSinceUs >= config.motion().settlingUs();
    }
    private boolean freshAlignment(Update update) {
        Alignment a = update.alignment();
        return a != null && targetId != null && a.targetId() == targetId && a.epoch() == epoch
                && SafetySupervisor.fresh(a.observationUs(), update.nowUs(), config.motion().maxAlignmentAgeUs())
                && a.uncertainty().maxVariance() <= config.motion().maxAlignmentVarianceM2();
    }
    private double alignmentHeadingError(Alignment a) {
        if (a.robotRelativeM().norm() == 0) return Math.PI;
        return angle(Math.atan2(a.robotRelativeM().y(), a.robotRelativeM().x())
                - Math.atan2(config.motion().intakePickupPointM().y(), config.motion().intakePickupPointM().x()));
    }
    private boolean aligned(Vec2 error, double headingError) {
        return error.norm() <= config.motion().alignmentToleranceM() && Math.abs(headingError) <= config.motion().alignmentHeadingToleranceRad();
    }
    private boolean startValid(Pose2 actual, Pose2 requested) {
        return actual.position().subtract(requested.position()).norm() <= config.planning().maxResultStartDisplacementM()
                && Math.abs(angle(actual.headingRad() - requested.headingRad())) <= config.planning().maxResultStartHeadingDriftRad();
    }
    /** O(1) endpoint validation; full solver path checking stays in the bounded worker, not this control path. */
    private boolean pathEndpointsValid(PlannerBackend.GeometricPath path, PlannerBackend.Request request) {
        if (path == null) return false;
        Pose2 start = path.points().get(0);
        Pose2 endpoint = path.points().get(path.points().size() - 1);
        PlannerBackend.Goal goal = request.goal();
        if (!startValid(start, request.startPose())) return false;
        return endpoint.position().subtract(goal.pose().position()).norm() <= goal.positionToleranceM()
                && Math.abs(angle(endpoint.headingRad() - goal.pose().headingRad())) <= goal.headingToleranceRad();
    }
    private void transition(State next, long nowUs, String why) { state = next; stateSinceUs = nowUs; detail = why; }
    private static boolean terminal(State state) { return state == State.SUCCEEDED || state == State.FAILED || state == State.CANCELLED || state == State.SAFE_STOP; }
    private Decision decision(World.WorldSnapshot snapshot, Update update, SafetySupervisor.Decision safety,
            Intent intent, PlannerBackend.Request request, Vec2 error, double headingError, boolean completed) {
        return new Decision(++nextDecisionId, taskId, state == State.IDLE ? snapshot.epoch() : epoch, snapshot.id(), update.nowUs(), state, intent, targetId, request,
                intent == Intent.FOLLOW_GEOMETRIC_PATH ? path : null, error, headingError, completed, attempts, recoveries, safety, detail);
    }
}
