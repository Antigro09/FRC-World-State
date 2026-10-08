package org.frcworldstate.core;

import java.util.List;
import java.util.Objects;
import java.util.Set;
import static org.frcworldstate.core.Geometry.*;

/**
 * Independent, bounded supervision of robot-owned facts and current collision envelopes.
 * Configuration values require robot measurements/qualification; this class supplies no
 * guessed "safe" defaults. Decisions are intents, never motor commands.
 */
public final class SafetySupervisor {
    public enum Action { ALLOW, HOLD, STOP }
    public enum Reason {
        CLEAR, DISABLED, LOST_DS, DRIVER_CANCEL, AUTHORITY_CONFLICT, UNSUPPORTED_MODE,
        INVALID_LOCALIZATION, STALE_REQUIRED_SENSING, COLLISION_INFORMATION_STALE,
        EPOCH_CHANGED, LOOP_OVERRUN, COLLISION_RISK
    }
    public record Config(long maxEgoAgeUs, long maxRequiredSensingAgeUs, long maxLoopDurationUs,
            double stoppingDecelerationMps2, double reactionLatencySec, double extraMarginM,
            double uncertaintySigma, PlannerBackend.Footprint footprint,
            boolean requireCollisionInformation, Action sensingLossAction, Set<String> allowedModes) {
        public Config {
            positive(maxEgoAgeUs); positive(maxRequiredSensingAgeUs); positive(maxLoopDurationUs);
            if (nonnegative(stoppingDecelerationMps2) == 0) throw new IllegalArgumentException("zero stopping deceleration");
            nonnegative(reactionLatencySec); nonnegative(extraMarginM); nonnegative(uncertaintySigma);
            Objects.requireNonNull(footprint); Objects.requireNonNull(sensingLossAction);
            if (sensingLossAction == Action.ALLOW) throw new IllegalArgumentException("required sensing loss must hold or stop");
            allowedModes = Set.copyOf(allowedModes);
            if (allowedModes.isEmpty()) throw new IllegalArgumentException("no permitted modes");
            allowedModes.forEach(Geometry::id);
        }
    }
    public record Input(long expectedEpoch, World.WorldSnapshot snapshot, World.RobotFacts facts,
            long nowUs, long loopDurationUs, List<PlannerBackend.Obstacle> obstacles,
            boolean collisionInformationFresh, PlannerBackend.Bounds bounds) {
        public Input {
            time(expectedEpoch); Objects.requireNonNull(snapshot); Objects.requireNonNull(facts);
            time(nowUs); time(loopDurationUs); obstacles = List.copyOf(obstacles);
            if (obstacles.size() > 4096) throw new IllegalArgumentException("obstacle capacity exceeded");
            Objects.requireNonNull(bounds);
        }
    }
    public record Decision(long id, long snapshotId, long epoch, long timeUs,
            Action action, Reason reason, double stoppingDistanceM, double envelopeRadiusM) {
        public Decision { time(id); time(snapshotId); time(epoch); time(timeUs); Objects.requireNonNull(action); Objects.requireNonNull(reason); nonnegative(stoppingDistanceM); nonnegative(envelopeRadiusM); }
        public boolean permitsMotion() { return action == Action.ALLOW; }
    }

    private final Config config;
    private long nextDecisionId;
    public SafetySupervisor(Config config) { this.config = Objects.requireNonNull(config); }
    public Config config() { return config; }

    /** Must also be called while following a previously accepted path. */
    public Decision evaluate(Input input) {
        World.RobotFacts facts = input.facts();
        if (input.snapshot().epoch() != input.expectedEpoch()) return decision(input, Action.STOP, Reason.EPOCH_CHANGED, 0, 0);
        if (!facts.dsConnected()) return decision(input, Action.STOP, Reason.LOST_DS, 0, 0);
        if (!facts.enabled()) return decision(input, Action.STOP, Reason.DISABLED, 0, 0);
        if (facts.driverCancel()) return decision(input, Action.STOP, Reason.DRIVER_CANCEL, 0, 0);
        if (!facts.commandAuthority()) return decision(input, Action.STOP, Reason.AUTHORITY_CONFLICT, 0, 0);
        if (!config.allowedModes().contains(facts.mode())) return decision(input, Action.STOP, Reason.UNSUPPORTED_MODE, 0, 0);
        if (!facts.ego().localizationValid() || !fresh(facts.ego().timeUs(), input.nowUs(), config.maxEgoAgeUs()))
            return decision(input, Action.STOP, Reason.INVALID_LOCALIZATION, 0, 0);
        if (input.loopDurationUs() > config.maxLoopDurationUs()) return decision(input, Action.STOP, Reason.LOOP_OVERRUN, 0, 0);
        if (!facts.requiredSensingFresh() || !fresh(facts.sensingTimeUs(), input.nowUs(), config.maxRequiredSensingAgeUs()))
            return decision(input, config.sensingLossAction(), Reason.STALE_REQUIRED_SENSING, 0, 0);
        if (config.requireCollisionInformation() && !input.collisionInformationFresh())
            return decision(input, config.sensingLossAction(), Reason.COLLISION_INFORMATION_STALE, 0, 0);

        double speed = facts.ego().fieldVelocity().linearMps().norm();
        double latencySec = config.reactionLatencySec() + (input.nowUs() - facts.ego().timeUs()) / 1_000_000.0;
        double stoppingDistance = speed * latencySec + speed * speed / (2 * config.stoppingDecelerationMps2());
        double radius = config.footprint().boundingRadiusM() + config.extraMarginM()
                + config.uncertaintySigma() * Math.sqrt(facts.ego().uncertainty().maxVariance());
        if (!Double.isFinite(stoppingDistance) || !Double.isFinite(radius))
            return decision(input, Action.STOP, Reason.INVALID_LOCALIZATION, 0, 0);
        double horizonUs = (latencySec + speed / config.stoppingDecelerationMps2()) * 1_000_000.0;
        if (!Double.isFinite(horizonUs) || horizonUs > Long.MAX_VALUE - input.nowUs())
            return decision(input, config.sensingLossAction(), Reason.COLLISION_INFORMATION_STALE, stoppingDistance, radius);
        long requiredUntilUs = input.nowUs() + (long) Math.ceil(horizonUs);
        for (PlannerBackend.Obstacle obstacle : input.obstacles()) {
            // An envelope must cover the whole stopping horizon, even if its last location is distant.
            if (obstacle.validUntilUs() < requiredUntilUs)
                return decision(input, config.sensingLossAction(), Reason.COLLISION_INFORMATION_STALE, stoppingDistance, radius);
        }
        Vec2 start = facts.ego().fieldPose().position();
        if (!inside(start, radius, input.bounds()))
            return decision(input, Action.STOP, Reason.COLLISION_RISK, stoppingDistance, radius);
        double endX = speed == 0 ? start.x() : start.x() + facts.ego().fieldVelocity().linearMps().x() * (stoppingDistance / speed);
        double endY = speed == 0 ? start.y() : start.y() + facts.ego().fieldVelocity().linearMps().y() * (stoppingDistance / speed);
        if (!Double.isFinite(endX) || !Double.isFinite(endY))
            return decision(input, Action.STOP, Reason.INVALID_LOCALIZATION, stoppingDistance, radius);
        Vec2 end = new Vec2(endX, endY);
        if (!inside(end, radius, input.bounds()))
            return decision(input, Action.STOP, Reason.COLLISION_RISK, stoppingDistance, radius);
        for (PlannerBackend.Obstacle obstacle : input.obstacles()) {
            if (distanceToSegment(obstacle.centerM(), start, end) <= radius + obstacle.envelopeRadiusM())
                return decision(input, Action.STOP, Reason.COLLISION_RISK, stoppingDistance, radius);
        }
        return decision(input, Action.ALLOW, Reason.CLEAR, stoppingDistance, radius);
    }

    private Decision decision(Input input, Action action, Reason reason, double distance, double radius) {
        return new Decision(++nextDecisionId, input.snapshot().id(), input.snapshot().epoch(), input.nowUs(), action, reason, distance, radius);
    }
    private static boolean inside(Vec2 position, double radius, PlannerBackend.Bounds b) {
        return position.x() - radius >= b.minXM() && position.x() + radius <= b.maxXM()
                && position.y() - radius >= b.minYM() && position.y() + radius <= b.maxYM();
    }
    private static double distanceToSegment(Vec2 point, Vec2 start, Vec2 end) {
        double x = end.x() - start.x(), y = end.y() - start.y();
        double length = Math.hypot(x, y);
        if (length == 0) return Math.hypot(point.x() - start.x(), point.y() - start.y());
        double ux = x / length, uy = y / length;
        double projection = (point.x() - start.x()) * ux + (point.y() - start.y()) * uy;
        double along = Math.max(0, Math.min(length, projection));
        double distance = Math.hypot(point.x() - (start.x() + ux * along), point.y() - (start.y() + uy * along));
        // Extreme finite coordinates must conservatively stop rather than escape comparison with NaN.
        return Double.isNaN(distance) ? 0 : distance;
    }
    static boolean fresh(long timestampUs, long nowUs, long maximumAgeUs) {
        return timestampUs <= nowUs && nowUs - timestampUs <= maximumAgeUs;
    }
    static void positive(long value) { if (value <= 0) throw new IllegalArgumentException("nonpositive duration"); }
}
