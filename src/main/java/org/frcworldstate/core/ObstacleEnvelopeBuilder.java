package org.frcworldstate.core;

import java.util.*;
import static org.frcworldstate.core.Geometry.*;

/** OFF-CONTROL-PATH conservative track-to-planner envelopes. Requires measured robot/season configuration. */
public final class ObstacleEnvelopeBuilder {
    public record ClassGeometry(double physicalRadiusM,double maximumMotionMps,boolean dynamic) {
        public ClassGeometry { if(nonnegative(physicalRadiusM)==0)throw new IllegalArgumentException("physical size must be explicitly positive");nonnegative(maximumMotionMps);if(!dynamic&&maximumMotionMps!=0)throw new IllegalArgumentException("fixed geometry cannot move"); }
    }
    public record Config(Map<String,ClassGeometry> classes,double uncertaintySigma,long maxMeasurementAgeUs,
            long maxSnapshotAgeUs,long maxEnvelopeHorizonUs,long maximumTimingErrorUs,double additionalLatencySec) {
        public Config {
            classes=Map.copyOf(classes);classes.keySet().forEach(Geometry::id);nonnegative(uncertaintySigma);nonnegative(additionalLatencySec);
            if(maxMeasurementAgeUs<=0||maxSnapshotAgeUs<=0||maxEnvelopeHorizonUs<=0||maximumTimingErrorUs<0)throw new IllegalArgumentException("invalid envelope bounds");
        }
    }
    public enum Status { COMPLETE, STALE_INFORMATION, UNKNOWN_GEOMETRY, INVALID_INPUT }
    public record Decision(Status status,List<PlannerBackend.Obstacle> obstacles,String detail) {
        public Decision { Objects.requireNonNull(status);obstacles=List.copyOf(obstacles);Objects.requireNonNull(detail);if(status!=Status.COMPLETE&&!obstacles.isEmpty())throw new IllegalArgumentException("partial envelopes cannot be used"); }
        public boolean usable() { return status==Status.COMPLETE; }
    }
    private final Config config;
    public ObstacleEnvelopeBuilder(Config config) { this.config=Objects.requireNonNull(config); }
    /** coverageFresh is independent authoritative collision sensing/coverage, never inferred from empty detections. */
    public Decision build(World.WorldSnapshot snapshot,long issuedUs,long validUntilUs,boolean coverageFresh) {
        Objects.requireNonNull(snapshot);time(issuedUs);time(validUntilUs);
        if(validUntilUs<=issuedUs||validUntilUs-issuedUs>config.maxEnvelopeHorizonUs()||snapshot.tracks().size()>256)
            return reject(Status.INVALID_INPUT,"invalid bounded validity interval");
        if(!coverageFresh||!snapshot.ego().localizationValid()||snapshot.ego().timeUs()>issuedUs||issuedUs-snapshot.ego().timeUs()>config.maxSnapshotAgeUs())
            return reject(Status.STALE_INFORMATION,"independent collision coverage/snapshot unavailable");
        List<PlannerBackend.Obstacle> envelopes=new ArrayList<>();Set<Long> ids=new HashSet<>();
        for(World.ObjectTrack track:snapshot.tracks()) {
            if(track.epoch()!=snapshot.epoch()||track.estimateUs()>issuedUs||!ids.add(track.id()))return reject(Status.INVALID_INPUT,"invalid track epoch/time/identity");
            if(issuedUs-track.lastMeasurementUs()>config.maxMeasurementAgeUs())return reject(Status.STALE_INFORMATION,"track last measurement stale: "+track.id());
            ClassGeometry geometry=config.classes().get(track.objectClass());
            if(geometry==null)return reject(Status.UNKNOWN_GEOMETRY,"no configured physical/motion bound: "+track.objectClass());
            if(track.velocityMps().norm()>geometry.maximumMotionMps())return reject(Status.INVALID_INPUT,"estimated speed exceeds configured motion envelope");
            // Published CP/CV estimate = last measured position + velocity * estimate age.
            Vec2 lastMeasuredCenter=track.positionM().subtract(track.velocityMps().scale((track.estimateUs()-track.lastMeasurementUs())/1e6));
            double uncertainty=config.uncertaintySigma()*Math.sqrt(track.uncertainty().maxVariance());
            double ageThroughExpiry=(validUntilUs-track.lastMeasurementUs())/1e6+config.maximumTimingErrorUs()/1e6+config.additionalLatencySec();
            double motion=geometry.maximumMotionMps()*ageThroughExpiry;
            double margin=uncertainty+motion;
            if(!Double.isFinite(margin))return reject(Status.INVALID_INPUT,"nonfinite computed envelope");
            // Physical radius is separate from uncertainty+age/motion. Backend adds raw robot footprint once.
            envelopes.add(new PlannerBackend.Obstacle("track:"+snapshot.epoch()+":"+track.id(),lastMeasuredCenter,
                    geometry.physicalRadiusM(),margin,validUntilUs,geometry.dynamic()));
        }
        return new Decision(Status.COMPLETE,envelopes,"physical size + positional uncertainty + capture age + bounded motion/timing/latency through expiry");
    }
    private static Decision reject(Status status,String detail) { return new Decision(status,List.of(),detail); }
}
