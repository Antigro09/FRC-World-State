package org.frcworldstate.core;

import java.util.Objects;
import java.util.Optional;
import static org.frcworldstate.core.Geometry.*;

/**
 * Required bindings and measured-feedback preflight for the existing pickup controller.
 * This is not a scheduler or follower. All ports must read cached robot-owned measurements
 * or accept bounded intents; no networking, blocking I/O or motor implementation belongs here.
 */
public final class PickupBindings {
    public record Evidence(String sourceId, long observedUs, boolean valid, boolean value) {
        public Evidence { id(sourceId); time(observedUs); }
    }
    @FunctionalInterface public interface EvidencePort { Evidence read(); }
    public interface FollowerIntentPort {
        void accept(PickupController.Decision intent);
        void stop(String reason);
    }
    public interface MechanismIntentPort {
        void accept(PickupController.Decision intent);
        void stop(String reason);
    }
    public enum Status { READY, ROBOT_FACTS_UNAVAILABLE, MECHANISM_UNAVAILABLE,
        POSSESSION_UNAVAILABLE, STALE_MECHANISM, STALE_POSSESSION, FACTS_DISAGREE }
    /** Only READY feedback contains facts usable for a controller update. */
    public record Feedback(Status status, long checkedUs, Optional<World.RobotFacts> facts, String detail) {
        public Feedback {
            Objects.requireNonNull(status); time(checkedUs); Objects.requireNonNull(facts); Objects.requireNonNull(detail);
            if ((status == Status.READY) != facts.isPresent()) throw new IllegalArgumentException("invalid checked feedback");
        }
    }

    private final PickupController controller;
    private final World.RobotPort robot;
    private final EvidencePort mechanism;
    private final EvidencePort possession;
    private final FollowerIntentPort follower;
    private final MechanismIntentPort mechanismIntents;
    private final long maximumEvidenceAgeUs;

    public PickupBindings(PickupController.Config configuration, SafetySupervisor supervisor,
            PickupController.ReachableScorer reachableScorer, World.RobotPort robot,
            EvidencePort mechanism, EvidencePort possession, FollowerIntentPort follower,
            MechanismIntentPort mechanismIntents, long maximumEvidenceAgeUs) {
        require(configuration, "pickup configuration including intake geometry, footprint, constraints and bounds");
        require(supervisor, "independent safety supervisor");
        require(reachableScorer, "bounded reachability scorer");
        this.robot = require(robot, "authoritative robot facts port");
        this.mechanism = require(mechanism, "measured mechanism feedback port");
        this.possession = require(possession, "measured possession feedback port");
        this.follower = require(follower, "follower intent port (no follower is supplied by this library)");
        this.mechanismIntents = require(mechanismIntents, "mechanism intent port");
        SafetySupervisor.positive(maximumEvidenceAgeUs);
        this.maximumEvidenceAgeUs = maximumEvidenceAgeUs;
        controller = new PickupController(configuration, supervisor, reachableScorer);
    }

    /** Reuses the sole task lifecycle implementation; the robot remains its owner. */
    public PickupController controller() { return controller; }
    public FollowerIntentPort followerIntentPort() { return follower; }
    public MechanismIntentPort mechanismIntentPort() { return mechanismIntents; }

    /**
     * A false, fresh possession measurement is valid evidence of unsuccessful acquisition.
     * An absent, invalid or stale measurement is a binding/sensing failure. It never becomes
     * a guessed false or success value. The helper checks agreement and does not rewrite facts.
     * Failure sends stop intents to both ports, including while an older command is following.
     */
    public Feedback readFeedback(long nowUs) {
        time(nowUs);
        World.RobotFacts facts;
        Evidence ready;
        Evidence possessed;
        try {
            facts = robot.read();
            ready = mechanism.read();
            possessed = possession.read();
        } catch (RuntimeException failure) {
            try { cancelAndStop(nowUs, "robot feedback port failed"); }
            catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
        if (facts == null) return stop(Status.ROBOT_FACTS_UNAVAILABLE, nowUs, "robot facts unavailable");
        if (ready == null || !ready.valid()) return stop(Status.MECHANISM_UNAVAILABLE, nowUs, "measured mechanism evidence unavailable");
        if (possessed == null || !possessed.valid()) return stop(Status.POSSESSION_UNAVAILABLE, nowUs, "measured possession evidence unavailable");
        if (!SafetySupervisor.fresh(ready.observedUs(), nowUs, maximumEvidenceAgeUs))
            return stop(Status.STALE_MECHANISM, nowUs, "mechanism evidence stale or future");
        if (!SafetySupervisor.fresh(possessed.observedUs(), nowUs, maximumEvidenceAgeUs))
            return stop(Status.STALE_POSSESSION, nowUs, "possession evidence stale or future");
        if (facts.mechanismReady() != ready.value() || facts.possessionVerified() != possessed.value())
            return stop(Status.FACTS_DISAGREE, nowUs, "authoritative robot facts and measured evidence disagree");
        return new Feedback(Status.READY, nowUs, Optional.of(facts), "measured feedback present and agrees with robot facts");
    }

    private Feedback stop(Status status, long nowUs, String detail) {
        cancelAndStop(nowUs, detail);
        return new Feedback(status, nowUs, Optional.empty(), detail);
    }
    private void cancelAndStop(long nowUs, String reason) {
        RuntimeException failure = null;
        try { controller.cancel(nowUs); } catch (RuntimeException exception) { failure = exception; }
        try { stopPorts(reason); }
        catch (RuntimeException exception) { if (failure == null) failure = exception; else failure.addSuppressed(exception); }
        if (failure != null) throw failure;
    }
    private void stopPorts(String reason) {
        // Attempt both even if a robot-owned port throws; retain the original failure.
        RuntimeException failure = null;
        try { follower.stop(reason); } catch (RuntimeException exception) { failure = exception; }
        try { mechanismIntents.stop(reason); }
        catch (RuntimeException exception) { if (failure == null) failure = exception; else failure.addSuppressed(exception); }
        if (failure != null) throw failure;
    }
    private static <T> T require(T value, String name) {
        if (value == null) throw new IllegalArgumentException("Missing binding: " + name);
        return value;
    }
}
