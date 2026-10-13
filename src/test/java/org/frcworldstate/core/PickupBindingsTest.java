package org.frcworldstate.core;

import java.util.Set;
import static org.frcworldstate.core.Geometry.*;

/** Cached synthetic feedback and recording intent ports; no hardware or follower implementation. */
public final class PickupBindingsTest {
    private PickupBindingsTest() {}
    private static int checks;
    private static final long NOW = 100_000;
    private static final World.EgoState EGO = new World.EgoState(NOW, new Pose2(new Vec2(2, 2), 0),
            Velocity2.zero(), new Uncertainty(.01, 0, .01), true);
    private static final World.RobotFacts FACTS = new World.RobotFacts(EGO, true, true, "AUTO", false, true,
            true, false, true, NOW);
    private static final class Ports implements PickupBindings.FollowerIntentPort, PickupBindings.MechanismIntentPort {
        int stops, accepted;
        @Override public void accept(PickupController.Decision intent) { accepted++; }
        @Override public void stop(String reason) { if (reason.isBlank()) throw new AssertionError("missing stop reason"); stops++; }
    }
    private static PickupController.Config config() {
        var footprint = new PlannerBackend.Footprint(.4, .4);
        var motion = new PickupController.Motion(new Vec2(.5, 0), .05, .1, .05, .1, 20_000, 30_000, .05, .1, .01);
        var planning = new PickupController.Planning(footprint, new PlannerBackend.Constraints(2, 2, 2),
                new PlannerBackend.Bounds(0, 0, 20, 20), 100_000_000, .15, .1);
        return new PickupController.Config("piece", .7, 500_000, 100_000, 2_000_000, 50_000, .2,
                5_000_000, 200_000, 150_000, 150_000, 20_000, 2, 1, .3, motion, planning);
    }
    private static SafetySupervisor safety() {
        return new SafetySupervisor(new SafetySupervisor.Config(100_000, 100_000, 25_000, 1, .1, .05, 2,
                config().planning().footprint(), true, SafetySupervisor.Action.HOLD, Set.of("AUTO")));
    }
    private static PickupBindings create(World.RobotPort robot, PickupBindings.EvidencePort mechanism,
            PickupBindings.EvidencePort possession, Ports ports) {
        return new PickupBindings(config(), safety(), (track, snapshot) -> new PickupController.CandidateScore(false, 0),
                robot, mechanism, possession, ports, ports, 30_000);
    }
    private static PickupBindings.Evidence evidence(boolean value, long stamp) {
        return new PickupBindings.Evidence("scripted-measured-feedback", stamp, true, value);
    }
    private static void missing(Runnable action, String binding) {
        try { action.run(); throw new AssertionError("missing binding was silently accepted: " + binding); }
        catch (IllegalArgumentException expected) { check(expected.getMessage().contains(binding), "named binding failure: " + binding); }
    }
    public static void run() {
        Ports requiredPorts = new Ports();
        var scorer = (PickupController.ReachableScorer) (track, snapshot) -> new PickupController.CandidateScore(false, 0);
        missing(() -> new PickupBindings(null, safety(), scorer, () -> FACTS, () -> evidence(true, NOW),
                () -> evidence(false, NOW), requiredPorts, requiredPorts, 30_000), "geometry");
        missing(() -> new PickupBindings(config(), safety(), scorer, () -> FACTS, () -> evidence(true, NOW),
                null, requiredPorts, requiredPorts, 30_000), "possession");
        missing(() -> new PickupBindings(config(), safety(), scorer, () -> FACTS, () -> evidence(true, NOW),
                () -> evidence(false, NOW), null, requiredPorts, 30_000), "follower");
        missing(() -> new PickupBindings(config(), safety(), null, () -> FACTS, () -> evidence(true, NOW),
                () -> evidence(false, NOW), requiredPorts, requiredPorts, 30_000), "reachability");

        Ports ports = new Ports();
        var bindings = create(() -> FACTS, () -> evidence(true, NOW), () -> evidence(false, NOW), ports);
        var read = bindings.readFeedback(NOW);
        check(read.status() == PickupBindings.Status.READY && read.facts().orElseThrow() == FACTS,
                "fresh false possession remains measured unsuccessful acquisition, preserving facts exactly");
        check(ports.stops == 0 && ports.accepted == 0, "checking ready bindings never actuates or starts following");
        check(bindings.controller().state() == PickupController.State.IDLE, "single existing controller remains idle");

        ports = new Ports();
        var noPossession = create(() -> FACTS, () -> evidence(true, NOW), () -> null, ports);
        noPossession.controller().start("missing-evidence", 0, NOW);
        read = noPossession.readFeedback(NOW);
        check(read.status() == PickupBindings.Status.POSSESSION_UNAVAILABLE && read.facts().isEmpty(), "absence is not guessed false");
        check(ports.stops == 2 && noPossession.controller().state() == PickupController.State.CANCELLED,
                "loss cancels task and sends both stop intents, including previously active commands");

        ports = new Ports();
        var stale = create(() -> FACTS, () -> evidence(true, NOW), () -> evidence(false, NOW - 30_001), ports);
        check(stale.readFeedback(NOW).status() == PickupBindings.Status.STALE_POSSESSION && ports.stops == 2, "stale evidence stops");
        ports = new Ports();
        var future = create(() -> FACTS, () -> evidence(true, NOW + 1), () -> evidence(false, NOW), ports);
        check(future.readFeedback(NOW).status() == PickupBindings.Status.STALE_MECHANISM && ports.stops == 2, "future evidence stops");
        ports = new Ports();
        var disagree = create(() -> FACTS, () -> evidence(true, NOW), () -> evidence(true, NOW), ports);
        check(disagree.readFeedback(NOW).status() == PickupBindings.Status.FACTS_DISAGREE && ports.stops == 2, "conflict cannot overwrite robot facts");
        ports = new Ports();
        var unavailable = create(() -> null, () -> evidence(true, NOW), () -> evidence(false, NOW), ports);
        check(unavailable.readFeedback(NOW).status() == PickupBindings.Status.ROBOT_FACTS_UNAVAILABLE && ports.stops == 2,
                "missing authoritative facts stops");
        Ports failedPorts = new Ports();
        RuntimeException portFailure = new IllegalStateException("scripted feedback error");
        var failed = create(() -> FACTS, () -> { throw portFailure; }, () -> evidence(false, NOW), failedPorts);
        try { failed.readFeedback(NOW); throw new AssertionError("port failure hidden"); }
        catch (RuntimeException expected) { check(expected == portFailure && failedPorts.stops == 2, "original error retained after stop intents"); }
        System.out.println("PickupBindingsTest: " + checks + " checks passed");
    }
    private static void check(boolean condition, String detail) { checks++; if (!condition) throw new AssertionError(detail); }
}
