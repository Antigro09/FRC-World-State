package org.frcworldstate.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;
import java.util.function.LongSupplier;
import static org.frcworldstate.core.Geometry.*;

/**
 * Bounded, deterministic, OFF-PERIODIC target selection. Each candidate calls the existing
 * injected planner and the core geometric validator. This class owns neither a scheduler
 * nor a follower. Use one instance on a serialized selection worker. A selected goal does
 * not imply collection, arrival or task completion.
 *
 * Cluster mode considers disks centered at eligible persistent track positions, with the
 * configured collection radius. It returns the best validated approach among that finite
 * candidate set; it does not solve unrestricted disk packing. Utility is a sum of configured
 * class weights for unique eligible physical track identities, never a camera detection count.
 * The disk describes a configured useful collection region, not proof that an intake will
 * collect every member. Robot execution, fresh relative alignment and possession remain required.
 */
public final class TargetSelector {
    public enum Mode { SELECTED_OBJECT, USEFUL_CLUSTER }
    public enum Status {
        SELECTED, NO_ELIGIBLE_TRACKS, SELECTED_TRACK_UNAVAILABLE, NO_REACHABLE_APPROACH,
        MISSING_CONFIGURATION, MISSING_SAFETY_SUPERVISOR, MISSING_PLANNER, MISSING_CLOCK,
        INVALID_INPUT, UNSAFE, CAPACITY_LIMIT, STALE_REQUEST, TIMEOUT, CANCELLED
    }
    public record TrackKey(long epoch, long id) {
        public TrackKey { time(epoch); time(id); }
    }
    public record Quality(long maximumMeasurementAgeUs, double minimumConfidence,
            double maximumVarianceM2, double coverageUncertaintySigma) {
        public Quality {
            SafetySupervisor.positive(maximumMeasurementAgeUs); nonnegative(minimumConfidence);
            if (minimumConfidence > 1) throw new IllegalArgumentException("confidence > 1");
            nonnegative(maximumVarianceM2); nonnegative(coverageUncertaintySigma);
        }
    }
    /** SI field disk and robot-relative intake point; angles are offsets from the nearest approach. */
    public record CollectionGeometry(double collectionRadiusM, Vec2 intakePickupPointM,
            List<Double> approachHeadingOffsetsRad, double positionToleranceM,
            double headingToleranceRad, double velocityToleranceMps) {
        public CollectionGeometry {
            if (nonnegative(collectionRadiusM) == 0) throw new IllegalArgumentException("zero collection radius");
            Objects.requireNonNull(intakePickupPointM, "missing intake pickup point");
            if (intakePickupPointM.norm() == 0) throw new IllegalArgumentException("intake pickup point must define a direction");
            approachHeadingOffsetsRad = List.copyOf(approachHeadingOffsetsRad);
            if (approachHeadingOffsetsRad.isEmpty() || approachHeadingOffsetsRad.size() > 8)
                throw new IllegalArgumentException("configure between 1 and 8 approach angles");
            for (Double a : approachHeadingOffsetsRad) finite(a);
            if (approachHeadingOffsetsRad.stream().map(Geometry::angle).distinct().count() != approachHeadingOffsetsRad.size())
                throw new IllegalArgumentException("duplicate normalized approach angles");
            nonnegative(positionToleranceM); nonnegative(headingToleranceRad); nonnegative(velocityToleranceMps);
        }
    }
    /** Confidence gates eligibility only. A score is not calibrated uncertainty. */
    public record Ranking(Map<String, Double> classUtilityWeights, double travelCostPerM,
            double switchingImprovement, long minimumLockUs) {
        public Ranking {
            classUtilityWeights = Map.copyOf(classUtilityWeights);
            if (classUtilityWeights.isEmpty()) throw new IllegalArgumentException("missing class utility weights");
            classUtilityWeights.forEach((k, v) -> { id(k); nonnegative(v); });
            if (classUtilityWeights.values().stream().noneMatch(v -> v > 0))
                throw new IllegalArgumentException("no useful object class");
            nonnegative(travelCostPerM); nonnegative(switchingImprovement); time(minimumLockUs);
        }
    }
    /** Footprint and limits must be robot configuration, not selector guesses. */
    public record Planning(PlannerBackend.Footprint footprint, PlannerBackend.Constraints constraints,
            PlannerBackend.Bounds bounds, int maximumCandidateEvaluations, int maximumProvenancePerTrack) {
        public Planning {
            if (maximumCandidateEvaluations <= 0 || maximumCandidateEvaluations > 2048)
                throw new IllegalArgumentException("candidate evaluation limit must be 1..2048");
            if (maximumProvenancePerTrack <= 0 || maximumProvenancePerTrack > 256)
                throw new IllegalArgumentException("track provenance limit must be 1..256");
        }
    }
    /** Partial binding is permitted so missing prerequisites produce a typed outcome. */
    public record Config(Quality quality, CollectionGeometry geometry, Ranking ranking, Planning planning) {}

    /**
     * Snapshot identity binds BLUE_FIELD, epoch and obstacle map. Obstacles must conservatively
     * cover the complete validity interval; an empty camera frame cannot establish coverageFresh.
     * The budget is the shared System.nanoTime domain budget for candidate generation, ALL
     * backend calls and validation. This method may only be called on a bounded worker.
     */
    public record Request(String requestId, String taskId, Mode mode, Long selectedTrackId,
            World.WorldSnapshot snapshot, World.RobotFacts facts, long nowUs, long validUntilUs,
            long loopDurationUs, List<PlannerBackend.Obstacle> obstacles, boolean coverageFresh,
            PlannerBackend.SearchBudget budget, PlannerBackend.Cancellation cancellation) {
        public Request {
            id(requestId); id(taskId); Objects.requireNonNull(mode); Objects.requireNonNull(snapshot);
            Objects.requireNonNull(facts); time(nowUs); time(validUntilUs); time(loopDurationUs);
            if (validUntilUs <= nowUs) throw new IllegalArgumentException("expired selection request");
            if (mode == Mode.SELECTED_OBJECT && selectedTrackId == null)
                throw new IllegalArgumentException("selected-object mode requires a persistent track id");
            if (mode == Mode.USEFUL_CLUSTER && selectedTrackId != null)
                throw new IllegalArgumentException("cluster mode cannot carry a selected object id");
            if (selectedTrackId != null) time(selectedTrackId);
            obstacles = List.copyOf(obstacles);
            if (obstacles.size() > 4096) throw new IllegalArgumentException("obstacle capacity exceeded");
            Objects.requireNonNull(budget); Objects.requireNonNull(cancellation);
        }
    }
    /** Distinct identities and bounded original observation provenance are retained for replay. */
    public record Member(TrackKey key, String objectClass, long lastMeasurementUs,
            World.TrackLifecycle lifecycle, World.EstimateKind estimateKind, long estimateUs,
            double utilityWeight, List<World.SourceStamp> provenance) {
        public Member {
            Objects.requireNonNull(key); id(objectClass); time(lastMeasurementUs); nonnegative(utilityWeight);
            Objects.requireNonNull(lifecycle); Objects.requireNonNull(estimateKind); time(estimateUs);
            if (lastMeasurementUs > estimateUs) throw new IllegalArgumentException("measurement after estimate");
            provenance = List.copyOf(provenance);
        }
    }
    /** A planner's SUCCESS is geometric reachability under this request, never physical success. */
    public record ReachabilityEvidence(String requestId, String backendId, long solverDurationNanos, long validUntilUs) {
        public ReachabilityEvidence { id(requestId); id(backendId); time(solverDurationNanos); time(validUntilUs); }
    }
    public record Selection(String requestId, String taskId, Mode mode, long epoch, long snapshotId,
            long obstacleMapVersion, FieldIdentity field, long issuedUs, long validUntilUs,
            TrackKey anchor, Vec2 collectionCenterM, double collectionRadiusM, List<Member> members,
            PlannerBackend.Goal goal, double utility, double travelDistanceM, double score,
            ReachabilityEvidence evidence, double robotClearanceMarginM, PlannerBackend.Bounds effectivePlanningBounds,
            boolean retainedByHysteresis) {
        public Selection {
            id(requestId); id(taskId); Objects.requireNonNull(mode); time(epoch); time(snapshotId);
            time(obstacleMapVersion); Objects.requireNonNull(field); time(issuedUs); time(validUntilUs);
            if (validUntilUs <= issuedUs) throw new IllegalArgumentException("expired selection");
            Objects.requireNonNull(anchor); Objects.requireNonNull(collectionCenterM); nonnegative(collectionRadiusM);
            members = List.copyOf(members); if (members.isEmpty()) throw new IllegalArgumentException("empty selection");
            Objects.requireNonNull(goal); nonnegative(utility); nonnegative(travelDistanceM); finite(score);
            Objects.requireNonNull(evidence); nonnegative(robotClearanceMarginM); Objects.requireNonNull(effectivePlanningBounds);
        }
        /** Exact admission gate for a worker reply. New snapshots need a new selection. */
        public boolean matches(World.WorldSnapshot current, String currentTaskId, long nowUs) {
            return taskId.equals(currentTaskId) && epoch == current.epoch() && snapshotId == current.id()
                && obstacleMapVersion == current.obstacleMapVersion() && field.equals(current.field())
                && nowUs >= issuedUs && nowUs < validUntilUs;
        }
    }
    public record Evaluation(TrackKey anchor, int approachIndex, PlannerBackend.Status status,
            String backendId, String detail) {
        public Evaluation { Objects.requireNonNull(anchor); Objects.requireNonNull(status); id(backendId); Objects.requireNonNull(detail); }
    }
    public record Outcome(Status status, Selection selection, List<Evaluation> evaluations, String detail) {
        public Outcome {
            Objects.requireNonNull(status); evaluations = List.copyOf(evaluations); Objects.requireNonNull(detail);
            if ((status == Status.SELECTED) != (selection != null)) throw new IllegalArgumentException("selection/status mismatch");
        }
    }

    private record Candidate(World.ObjectTrack anchor, List<World.ObjectTrack> members,
            int approachIndex, PlannerBackend.Goal goal, double utility) {}
    private record Scored(Candidate candidate, double distanceM, double score,
            PlannerBackend.Request request, PlannerBackend.Result result) {}
    private record Lock(String taskId, Mode mode, Long explicitTrackId, long epoch,
            long mapVersion, FieldIdentity field, TrackKey anchor, int approachIndex, long sinceUs,
            long latestSnapshotId, long latestEvaluationUs) {}

    private final Config config;
    private final SafetySupervisor supervisor;
    private final PlannerBackend planner;
    private final LongSupplier robotTimeUs;
    private Lock lock;

    public TargetSelector(Config config, SafetySupervisor supervisor, PlannerBackend planner, LongSupplier robotTimeUs) {
        // Copy immutable safety configuration into a private supervisor so its decision counter
        // is never concurrently shared with the periodic execution supervisor.
        this.config = config; this.supervisor = supervisor == null ? null : new SafetySupervisor(supervisor.config());
        this.planner = planner; this.robotTimeUs = robotTimeUs;
    }
    /** Explicit task cancellation/replacement may release the local selection lock. */
    public void reset() { lock = null; }

    public Outcome select(Request request) {
        Objects.requireNonNull(request);
        try { return selectBounded(request); }
        catch (IllegalArgumentException | ArithmeticException ex) {
            return reject(Status.INVALID_INPUT, "invalid or overflowing configured geometry: " + ex.getClass().getSimpleName());
        }
    }

    private Outcome selectBounded(Request request) {
        if (!sameContext(request)) lock = null;
        else if (request.snapshot().id() < lock.latestSnapshotId() || request.nowUs() < lock.latestEvaluationUs())
            return reject(Status.STALE_REQUEST, "older queued request cannot replace the current selection lock");
        if (config == null || config.quality() == null || config.geometry() == null || config.ranking() == null
                || config.planning() == null || config.planning().footprint() == null
                || config.planning().constraints() == null || config.planning().bounds() == null)
            return reject(Status.MISSING_CONFIGURATION, "quality, collection geometry, class weights, footprint, bounds and motion constraints are required");
        if (supervisor == null) return reject(Status.MISSING_SAFETY_SUPERVISOR, "independent robot safety supervision is required");
        if (planner == null) return reject(Status.MISSING_PLANNER, "a bounded planner backend is required; distance is not a reachability proof");
        if (robotTimeUs == null) return reject(Status.MISSING_CLOCK, "robot monotonic microsecond clock is required");
        if (!config.planning().footprint().equals(supervisor.config().footprint()))
            return reject(Status.INVALID_INPUT, "selection and safety footprints differ");
        Status live = live(request);
        if (live != null) return reject(live, "selection cancelled, budget exceeded, or robot clock outside request validity");
        long nowUs = robotTimeUs.getAsLong();
        if (!request.snapshot().ego().equals(request.facts().ego()))
            return reject(Status.STALE_REQUEST, "snapshot and authoritative robot ego feedback differ");
        if (!request.coverageFresh()) return reject(Status.UNSAFE, "collision coverage is unknown or stale");
        for (PlannerBackend.Obstacle obstacle : request.obstacles()) {
            live = live(request); if (live != null) return reject(live, "obstacle validation cancelled or expired");
            if (obstacle.validUntilUs() < request.validUntilUs())
                return reject(Status.UNSAFE, "collision envelope does not cover the request validity interval");
        }
        SafetySupervisor.Decision safety = supervisor.evaluate(new SafetySupervisor.Input(request.snapshot().epoch(),
                request.snapshot(), request.facts(), nowUs, request.loopDurationUs(), request.obstacles(),
                request.coverageFresh(), config.planning().bounds()));
        if (!safety.permitsMotion()) return reject(Status.UNSAFE, "robot safety: " + safety.reason());
        if (request.facts().ego().fieldVelocity().linearMps().norm() > config.planning().constraints().maxSpeedMps()
                || Math.abs(request.facts().ego().fieldVelocity().angularRadPerSec()) > config.planning().constraints().maxAngularSpeedRadPerSec())
            return reject(Status.INVALID_INPUT, "authoritative start velocity exceeds configured motion constraints");

        // Preserve raw physical footprint. Add the explicitly configured current-ego margin
        // once to occupancy envelopes and shrink bounds once for every reachability request.
        // This is not a qualification of future covariance growth or stopping dynamics.
        double robotMarginM = supervisor.config().extraMarginM() + supervisor.config().uncertaintySigma()
            * Math.sqrt(request.facts().ego().uncertainty().maxVariance());
        PlannerBackend.Bounds physicalBounds = config.planning().bounds();
        PlannerBackend.Bounds effectiveBounds = new PlannerBackend.Bounds(physicalBounds.minXM() + robotMarginM,
            physicalBounds.minYM() + robotMarginM, physicalBounds.maxXM() - robotMarginM, physicalBounds.maxYM() - robotMarginM);
        List<PlannerBackend.Obstacle> effectiveObstacles = new ArrayList<>();
        for (PlannerBackend.Obstacle obstacle : request.obstacles()) {
            live = live(request); if (live != null) return reject(live, "robot-clearance derivation cancelled or expired");
            effectiveObstacles.add(new PlannerBackend.Obstacle(obstacle.id(), obstacle.centerM(), obstacle.radiusM(),
                obstacle.uncertaintyMarginM() + robotMarginM, obstacle.validUntilUs(), obstacle.dynamic()));
        }

        TreeMap<Long, World.ObjectTrack> unique = new TreeMap<>();
        for (World.ObjectTrack track : request.snapshot().tracks()) {
            live = live(request); if (live != null) return reject(live, "track validation cancelled or expired");
            World.ObjectTrack previous = unique.putIfAbsent(track.id(), track);
            // Ignore exact duplicates only. A fresher conflicting duplicate cannot rehabilitate bad quality.
            if (previous != null && !previous.equals(track))
                return reject(Status.INVALID_INPUT, "conflicting duplicate persistent track identity " + track.id());
            if (track.provenance().size() > config.planning().maximumProvenancePerTrack())
                return reject(Status.CAPACITY_LIMIT, "track observation provenance exceeds configured bound");
        }
        List<World.ObjectTrack> eligible = new ArrayList<>();
        for (World.ObjectTrack track : unique.values()) if (eligible(track, request.snapshot().epoch(), nowUs)) eligible.add(track);
        if (request.mode() == Mode.SELECTED_OBJECT && eligible.stream().noneMatch(t -> t.id() == request.selectedTrackId()))
            return reject(Status.SELECTED_TRACK_UNAVAILABLE, "selected persistent track is absent, tentative, stale, uncertain or not useful");
        if (eligible.isEmpty()) return reject(Status.NO_ELIGIBLE_TRACKS, "no confirmed fresh useful tracks");

        List<Candidate> candidates = new ArrayList<>();
        for (World.ObjectTrack anchor : eligible) {
            live = live(request); if (live != null) return reject(live, "cluster generation cancelled or expired");
            if (request.mode() == Mode.SELECTED_OBJECT && anchor.id() != request.selectedTrackId()) continue;
            List<World.ObjectTrack> members = new ArrayList<>();
            if (request.mode() == Mode.SELECTED_OBJECT) {
                if (covered(anchor.positionM(), anchor)) members.add(anchor);
            } else for (World.ObjectTrack member : eligible) {
                live = live(request); if (live != null) return reject(live, "cluster generation cancelled or expired");
                if (covered(anchor.positionM(), member)) members.add(member);
            }
            if (members.isEmpty() || members.stream().noneMatch(t -> t.id() == anchor.id())) continue;
            double utility = 0;
            for (World.ObjectTrack member : members) utility += config.ranking().classUtilityWeights().get(member.objectClass());
            if (!Double.isFinite(utility)) return reject(Status.INVALID_INPUT, "utility sum overflow");
            Vec2 toward = anchor.positionM().subtract(request.facts().ego().fieldPose().position());
            double nearestHeading = toward.norm() == 0 ? request.facts().ego().fieldPose().headingRad()
                : Math.atan2(toward.y(), toward.x()) - Math.atan2(config.geometry().intakePickupPointM().y(), config.geometry().intakePickupPointM().x());
            for (int i = 0; i < config.geometry().approachHeadingOffsetsRad().size(); i++) {
                if (candidates.size() == config.planning().maximumCandidateEvaluations())
                    return reject(Status.CAPACITY_LIMIT, "anchor/approach candidate set exceeds bound; no partial greatest-cluster claim");
                double heading = angle(nearestHeading + config.geometry().approachHeadingOffsetsRad().get(i));
                Pose2 pose = new Pose2(anchor.positionM().subtract(config.geometry().intakePickupPointM().rotate(heading)), heading);
                PlannerBackend.Goal goal = new PlannerBackend.Goal(pose, config.geometry().positionToleranceM(),
                    config.geometry().headingToleranceRad(), config.geometry().velocityToleranceMps());
                candidates.add(new Candidate(anchor, List.copyOf(members), i, goal, utility));
            }
        }
        if (candidates.isEmpty()) return reject(Status.NO_ELIGIBLE_TRACKS, "track uncertainty cannot fit inside the configured collection region");
        List<Evaluation> evaluations = new ArrayList<>();
        Scored best = null, locked = null;
        for (Candidate candidate : candidates) {
            live = live(request); if (live != null) return reject(live, evaluations, "selection did not finish evaluating its bounded candidate set");
            if (!goalClear(candidate.goal(), request, effectiveBounds, effectiveObstacles)) {
                evaluations.add(new Evaluation(key(candidate.anchor()), candidate.approachIndex(), PlannerBackend.Status.INVALID_INPUT,
                    "core-geometry", "goal footprint intersects map bounds or a conservative collision envelope"));
                continue;
            }
            PlannerBackend.Request plan = new PlannerBackend.Request(request.requestId() + ":reach:" + candidate.anchor().id() + ":" + candidate.approachIndex(),
                request.taskId(), request.snapshot().epoch(), request.snapshot().id(), request.snapshot().obstacleMapVersion(), request.snapshot().field(),
                request.facts().ego().fieldPose(), request.facts().ego().fieldVelocity(), candidate.goal(), config.planning().footprint(),
                config.planning().constraints(), effectiveBounds, effectiveObstacles, request.nowUs(), request.validUntilUs(),
                request.budget(), request.cancellation());
            PlannerBackend.Result raw;
            try { raw = planner.plan(plan); }
            catch (RuntimeException ex) {
                evaluations.add(new Evaluation(key(candidate.anchor()), candidate.approachIndex(), PlannerBackend.Status.INVALID_INPUT,
                    "backend-exception", "backend failed: " + ex.getClass().getSimpleName()));
                continue;
            }
            live = live(request); if (live != null) return reject(live, evaluations, "planner evaluation cancelled or expired");
            PlannerBackend.Result checked = PlannerValidation.validate(plan, raw, request.snapshot(), request.snapshot().epoch(),
                request.snapshot().obstacleMapVersion(), robotTimeUs.getAsLong());
            evaluations.add(new Evaluation(key(candidate.anchor()), candidate.approachIndex(), checked.status(), checked.backendId(), checked.detail()));
            if (checked.status() == PlannerBackend.Status.CANCELLED) return reject(Status.CANCELLED, evaluations, "backend cancelled; no partial selection");
            if (checked.status() == PlannerBackend.Status.TIMEOUT) return reject(Status.TIMEOUT, evaluations, "backend timed out; no partial selection");
            if (checked.status() != PlannerBackend.Status.SUCCESS) continue;
            double distance = pathLength(checked.path(), request);
            double score = candidate.utility() - config.ranking().travelCostPerM() * distance;
            if (!Double.isFinite(distance) || !Double.isFinite(score)) {
                evaluations.set(evaluations.size() - 1, new Evaluation(key(candidate.anchor()), candidate.approachIndex(),
                    PlannerBackend.Status.INVALID_INPUT, "core-geometry", "nonfinite geometric path score"));
                continue;
            }
            Scored scored = new Scored(candidate, distance, score, plan, checked);
            if (better(scored, best)) best = scored;
            if (isLocked(candidate)) locked = scored;
        }
        live = live(request); if (live != null) return reject(live, evaluations, "selection expired before publication");
        if (best == null) { lock = null; return reject(Status.NO_REACHABLE_APPROACH, evaluations, "no approach has a validated geometric path"); }
        nowUs = robotTimeUs.getAsLong();
        SafetySupervisor.Decision finalSafety = supervisor.evaluate(new SafetySupervisor.Input(request.snapshot().epoch(),
            request.snapshot(), request.facts(), nowUs, request.loopDurationUs(), request.obstacles(), request.coverageFresh(), config.planning().bounds()));
        if (!finalSafety.permitsMotion()) return reject(Status.UNSAFE, evaluations, "robot safety after evaluation: " + finalSafety.reason());
        // A member can expire while the worker runs. Do not rebase or publish a stale candidate.
        for (World.ObjectTrack member : best.candidate().members())
            if (!eligible(member, request.snapshot().epoch(), nowUs)) return reject(Status.STALE_REQUEST, evaluations, "selected member aged out during evaluation");
        boolean retain = locked != null && (nowUs - lock.sinceUs() < config.ranking().minimumLockUs()
            || best.score() - locked.score() <= config.ranking().switchingImprovement());
        Scored chosen = retain ? locked : best;
        if (chosen.result().validUntilUs() <= nowUs)
            return reject(Status.STALE_REQUEST, evaluations, "chosen planner evidence expired during later candidate evaluation");
        for (World.ObjectTrack member : chosen.candidate().members())
            if (!eligible(member, request.snapshot().epoch(), nowUs)) return reject(Status.STALE_REQUEST, evaluations, "locked member aged out during evaluation");
        long expiryUs = Math.min(chosen.result().validUntilUs(), Math.min(request.validUntilUs(), Math.min(
            saturatedAdd(request.facts().ego().timeUs(), supervisor.config().maxEgoAgeUs()),
            saturatedAdd(request.facts().sensingTimeUs(), supervisor.config().maxRequiredSensingAgeUs()))));
        List<Member> members = new ArrayList<>();
        for (World.ObjectTrack member : chosen.candidate().members()) {
            long freshUntilUs = saturatedAdd(member.lastMeasurementUs(), config.quality().maximumMeasurementAgeUs());
            expiryUs = Math.min(expiryUs, freshUntilUs);
            members.add(new Member(key(member), member.objectClass(), member.lastMeasurementUs(),
                member.lifecycle(), member.kind(), member.estimateUs(),
                config.ranking().classUtilityWeights().get(member.objectClass()), member.provenance()));
        }
        if (expiryUs <= nowUs) return reject(Status.STALE_REQUEST, evaluations, "selection has no remaining measurement validity");
        live = live(request); if (live != null) return reject(live, evaluations, "selection cancelled or expired before final publication");
        if (robotTimeUs.getAsLong() >= expiryUs) return reject(Status.STALE_REQUEST, evaluations, "selection freshness expired before final publication");
        TrackKey anchor = key(chosen.candidate().anchor());
        long lockedSinceUs = retain ? lock.sinceUs() : nowUs;
        lock = new Lock(request.taskId(), request.mode(), request.selectedTrackId(), request.snapshot().epoch(),
            request.snapshot().obstacleMapVersion(), request.snapshot().field(), anchor, chosen.candidate().approachIndex(), lockedSinceUs,
            request.snapshot().id(), nowUs);
        Selection selection = new Selection(request.requestId(), request.taskId(), request.mode(), request.snapshot().epoch(),
            request.snapshot().id(), request.snapshot().obstacleMapVersion(), request.snapshot().field(), request.nowUs(), expiryUs,
            anchor, chosen.candidate().anchor().positionM(), config.geometry().collectionRadiusM(), members, chosen.candidate().goal(),
            chosen.candidate().utility(), chosen.distanceM(), chosen.score(),
            new ReachabilityEvidence(chosen.request().requestId(), chosen.result().backendId(), chosen.result().solverDurationNanos(), chosen.result().validUntilUs()),
            robotMarginM, effectiveBounds, retain);
        return new Outcome(Status.SELECTED, selection, evaluations, "best validated configured anchor approach; execution and possession are still required");
    }

    private boolean eligible(World.ObjectTrack track, long epoch, long nowUs) {
        return track.epoch() == epoch && track.lifecycle() != World.TrackLifecycle.TENTATIVE
            && track.estimateUs() <= nowUs && SafetySupervisor.fresh(track.lastMeasurementUs(), nowUs, config.quality().maximumMeasurementAgeUs())
            && track.confidence() >= config.quality().minimumConfidence()
            && track.uncertainty().maxVariance() <= config.quality().maximumVarianceM2()
            && config.ranking().classUtilityWeights().getOrDefault(track.objectClass(), 0.0) > 0;
    }
    private boolean covered(Vec2 center, World.ObjectTrack member) {
        double margin = config.quality().coverageUncertaintySigma() * Math.sqrt(member.uncertainty().maxVariance())
            + config.geometry().positionToleranceM();
        return member.positionM().subtract(center).norm() + margin <= config.geometry().collectionRadiusM();
    }
    private boolean goalClear(PlannerBackend.Goal goal, Request request, PlannerBackend.Bounds effectiveBounds,
            List<PlannerBackend.Obstacle> effectiveObstacles) {
        double radius = config.planning().footprint().boundingRadiusM();
        Vec2 p = goal.pose().position(); PlannerBackend.Bounds b = effectiveBounds;
        if (p.x() - radius < b.minXM() || p.y() - radius < b.minYM() || p.x() + radius > b.maxXM() || p.y() + radius > b.maxYM()) return false;
        for (PlannerBackend.Obstacle obstacle : effectiveObstacles) {
            if (live(request) != null) return false;
            if (p.subtract(obstacle.centerM()).norm() <= radius + obstacle.envelopeRadiusM()) return false;
        }
        return true;
    }
    private double pathLength(PlannerBackend.GeometricPath path, Request request) {
        double distance = 0; for (int i = 1; i < path.points().size(); i++) {
            if ((i & 31) == 0 && live(request) != null) return Double.NaN;
            distance += path.points().get(i).position().subtract(path.points().get(i - 1).position()).norm();
        }
        return distance;
    }
    private static boolean better(Scored candidate, Scored best) {
        if (best == null) return true;
        int score = Double.compare(candidate.score(), best.score());
        if (score != 0) return score > 0;
        int identity = Long.compare(candidate.candidate().anchor().id(), best.candidate().anchor().id());
        return identity < 0 || (identity == 0 && candidate.candidate().approachIndex() < best.candidate().approachIndex());
    }
    private boolean sameContext(Request r) {
        return lock != null && lock.taskId().equals(r.taskId()) && lock.mode() == r.mode()
            && Objects.equals(lock.explicitTrackId(), r.selectedTrackId()) && lock.epoch() == r.snapshot().epoch()
            && lock.mapVersion() == r.snapshot().obstacleMapVersion() && lock.field().equals(r.snapshot().field());
    }
    private boolean isLocked(Candidate c) { return lock != null && lock.anchor().equals(key(c.anchor())) && lock.approachIndex() == c.approachIndex(); }
    private Status live(Request r) {
        if (r.cancellation().cancelled()) return Status.CANCELLED;
        long elapsedNanos = System.nanoTime() - r.budget().startedNanos();
        if (elapsedNanos < 0) return Status.INVALID_INPUT;
        if (elapsedNanos >= r.budget().limitNanos()) return Status.TIMEOUT;
        long now = robotTimeUs.getAsLong();
        if (now < r.nowUs()) return Status.INVALID_INPUT;
        return now >= r.validUntilUs() ? Status.TIMEOUT : null;
    }
    private static TrackKey key(World.ObjectTrack t) { return new TrackKey(t.epoch(), t.id()); }
    private static long saturatedAdd(long a, long b) { return Long.MAX_VALUE - a < b ? Long.MAX_VALUE : a + b; }
    private static Outcome reject(Status status, String detail) { return reject(status, List.of(), detail); }
    private static Outcome reject(Status status, List<Evaluation> evaluations, String detail) { return new Outcome(status, null, evaluations, detail); }
}
