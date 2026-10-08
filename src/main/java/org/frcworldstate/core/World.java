package org.frcworldstate.core;

import java.util.List;
import java.util.Objects;
import static org.frcworldstate.core.Geometry.*;

/** Immutable authoritative state and normalized observations, independent of the raw vision wire schema. */
public final class World {
    private World() {}
    public record EgoState(long timeUs, Pose2 fieldPose, Velocity2 fieldVelocity,
                           Uncertainty uncertainty, boolean localizationValid) {
        public EgoState { time(timeUs); Objects.requireNonNull(fieldPose); Objects.requireNonNull(fieldVelocity); Objects.requireNonNull(uncertainty); }
    }
    public record SourceStamp(String sourceId, String bootId, String calibrationRevision,
            String mountRevision, String correlationGroup, long sequence, long captureUs,
            long publicationUs, double targetHeightM) {
        public SourceStamp {
            id(sourceId); id(bootId); id(calibrationRevision); id(mountRevision); id(correlationGroup);
            time(sequence); time(captureUs); time(publicationUs); finite(targetHeightM);
            if (publicationUs<captureUs) throw new IllegalArgumentException("publication before capture");
        }
    }
    public record Observation(String detectionId, String objectClass, double confidence,
            Vec2 robotRelativeM, Uncertainty relativeUncertainty) {
        public Observation {
            id(detectionId); id(objectClass); nonnegative(confidence);
            if (confidence>1) throw new IllegalArgumentException("confidence > 1");
            Objects.requireNonNull(robotRelativeM); Objects.requireNonNull(relativeUncertainty);
        }
    }
    /** Times already mapped to robot monotonic microseconds by the optional adapter. */
    public record ObservationFrame(long epoch, SourceStamp stamp, List<Observation> observations) {
        public ObservationFrame {
            time(epoch); Objects.requireNonNull(stamp); observations=List.copyOf(observations);
            if (observations.size()>256) throw new IllegalArgumentException("unbounded detections");
        }
    }
    public enum TrackLifecycle { TENTATIVE, CONFIRMED, COASTING }
    public enum EstimateKind { MEASURED, PROPAGATED }
    public record ObjectTrack(long id, long epoch, String objectClass, Vec2 positionM,
            Vec2 velocityMps, long estimateUs, long lastMeasurementUs, double confidence,
            Uncertainty uncertainty, TrackLifecycle lifecycle, EstimateKind kind,
            List<SourceStamp> provenance) {
        public ObjectTrack {
            time(id); time(epoch); Geometry.id(objectClass); time(estimateUs); time(lastMeasurementUs);
            Objects.requireNonNull(positionM); Objects.requireNonNull(velocityMps); Objects.requireNonNull(uncertainty);
            nonnegative(confidence); if (confidence>1||lastMeasurementUs>estimateUs) throw new IllegalArgumentException("invalid track");
            Objects.requireNonNull(lifecycle); Objects.requireNonNull(kind); provenance=List.copyOf(provenance);
        }
    }
    public record WorldSnapshot(long id, long epoch, long obstacleMapVersion, FieldIdentity field,
            EgoState ego, List<ObjectTrack> tracks) {
        public WorldSnapshot {
            time(id); time(epoch); time(obstacleMapVersion); Objects.requireNonNull(field); Objects.requireNonNull(ego);
            tracks=List.copyOf(tracks);
            if(tracks.size()>256)throw new IllegalArgumentException("snapshot track capacity exceeded");
            if(tracks.stream().anyMatch(t->t.epoch()!=epoch)) throw new IllegalArgumentException("track epoch mismatch");
        }
    }
    /** Robot owns these facts. Mechanism-ready and possession must come from measured robot feedback. */
    public record RobotFacts(EgoState ego, boolean enabled, boolean dsConnected,
            String mode, boolean driverCancel, boolean commandAuthority, boolean mechanismReady,
            boolean possessionVerified, boolean requiredSensingFresh, long sensingTimeUs) {
        public RobotFacts { Objects.requireNonNull(ego); id(mode); time(sensingTimeUs); }
    }
    @FunctionalInterface public interface RobotPort { RobotFacts read(); }
    public enum ResetReason { ROBOT_BOOT, FIELD_CHANGED, GEOMETRY_CHANGED, LOCALIZATION_HARD_RESET, RECONNECT }
}
