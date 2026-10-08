package org.frcworldstate.core;

import java.util.List;
import java.util.Objects;
import static org.frcworldstate.core.Geometry.*;

/** Optional bounded predictor SPI. Forecasts are advisory and never robot facts or motor commands. */
@FunctionalInterface
public interface Predictor {
    String SCHEMA_VERSION = "frc-prediction/1";
    long MAX_WIRE_INTEGER = 9_007_199_254_740_991L;
    int MAX_HISTORY = 128, MAX_TARGETS = 32, MAX_CANDIDATES = 16, MAX_SAMPLES = 256,
        MAX_HISTORY_TRACKS = 256, MAX_PROVENANCE = 256;
    Reply predict(Request request);
    @FunctionalInterface interface Cancellation { boolean cancelled(); }
    enum Frame { BLUE_FIELD, ROBOT_RELATIVE }
    enum TimestampDomain { ROBOT_MONOTONIC_US }
    enum TargetKind { EGO, OBJECT_TRACK }
    enum Authority { CANDIDATE, ACCEPTED }
    enum ActionSemantics { BODY_TWIST_OPEN_LOOP_V1 }
    enum ForecastKind { LEARNED_FORECAST }

    private static void wireInteger(long value) { time(value); if(value>MAX_WIRE_INTEGER) throw new IllegalArgumentException("wire integer outside exact JSON range"); }
    private static void wireId(String value) { id(value); if(value.codePointCount(0,value.length())>256) throw new IllegalArgumentException("wire identity too long"); }
    private static void wireField(FieldIdentity value) { Objects.requireNonNull(value); wireId(value.season()); wireId(value.mapId()); wireId(value.geometryRevision()); }

    record SourceContext(String robotBootId, String localizationRevision, String configurationRevision) {
        public SourceContext { wireId(robotBootId); wireId(localizationRevision); wireId(configurationRevision); }
    }
    record SyncMetadata(String sourceId, String mappingRevision, boolean valid, long maximumErrorUs) {
        public SyncMetadata { wireId(sourceId); wireId(mappingRevision); wireInteger(maximumErrorUs); }
    }
    record Target(String targetId, TargetKind kind, Long trackId) {
        public Target {
            wireId(targetId); Objects.requireNonNull(kind);
            if ((kind == TargetKind.EGO) != (trackId == null)) throw new IllegalArgumentException("target identity kind mismatch");
            if (trackId != null) wireInteger(trackId);
        }
    }
    /** Unit is a configured explicit SI unit or ratio/bool; bool values are 0 or 1. */
    record MechanismValue(String channel, String unit, double value) {
        public MechanismValue { wireId(channel); wireId(unit); }
    }
    /** Intended robot-relative chassis motion. Raw fields allow the gate to reject malformed model inputs. */
    record BodyTwist(double vxMps, double vyMps, double omegaRadps) {}
    record ActionSample(long offsetUs, BodyTwist bodyTwist, List<MechanismValue> mechanisms) {
        public ActionSample { wireInteger(offsetUs); Objects.requireNonNull(bodyTwist); mechanisms = List.copyOf(mechanisms); if(mechanisms.size()>32) throw new IllegalArgumentException("mechanism capacity"); }
    }
    record ActionCandidate(String candidateId, Authority authority, ActionSemantics semantics,
                           Frame frame, List<ActionSample> samples) {
        public ActionCandidate {
            wireId(candidateId); Objects.requireNonNull(authority); Objects.requireNonNull(semantics); Objects.requireNonNull(frame);
            samples = List.copyOf(samples);
            if(samples.isEmpty() || samples.size()>MAX_SAMPLES) throw new IllegalArgumentException("candidate capacity");
        }
    }
    /** Only the robot execution boundary may supply ACCEPTED authority. It is not measured motion. */
    record AcceptedCommand(String commandId, String sourceId, Authority authority, Frame frame,
                           long issuedUs, long acceptedUs, BodyTwist bodyTwist, List<MechanismValue> mechanisms) {
        public AcceptedCommand {
            wireId(commandId); wireId(sourceId); Objects.requireNonNull(authority); Objects.requireNonNull(frame);
            wireInteger(issuedUs); wireInteger(acceptedUs); Objects.requireNonNull(bodyTwist);
            mechanisms = List.copyOf(mechanisms); if(mechanisms.size()>32) throw new IllegalArgumentException("mechanism capacity");
        }
    }
    record HistorySample(World.WorldSnapshot snapshot, boolean validMask, AcceptedCommand acceptedCommand) {
        public HistorySample {
            Objects.requireNonNull(snapshot);
            if(snapshot.tracks().size()>MAX_HISTORY_TRACKS||snapshot.tracks().stream().anyMatch(t->t.provenance().size()>MAX_PROVENANCE)) throw new IllegalArgumentException("history track/provenance capacity");
            wireInteger(snapshot.id()); wireInteger(snapshot.epoch()); wireInteger(snapshot.obstacleMapVersion()); wireInteger(snapshot.ego().timeUs()); wireField(snapshot.field());
            for(World.ObjectTrack t:snapshot.tracks()) {
                wireInteger(t.id()); wireInteger(t.epoch()); wireInteger(t.estimateUs()); wireInteger(t.lastMeasurementUs()); wireId(t.objectClass());
                for(World.SourceStamp s:t.provenance()) { wireId(s.sourceId()); wireId(s.bootId()); wireId(s.calibrationRevision()); wireId(s.mountRevision()); wireId(s.correlationGroup()); wireInteger(s.sequence()); wireInteger(s.captureUs()); wireInteger(s.publicationUs()); }
            }
        }
    }
    record Request(String schemaVersion, String modelId, String requestId, long snapshotId,
                   long epoch, FieldIdentity field, SourceContext source, Frame frame,
                   TimestampDomain timestampDomain, SyncMetadata sync, long issuedUs,
                   long historyCutoffUs, long validUntilUs, long horizonUs, long stepUs,
                   List<HistorySample> history, List<Target> targets,
                   List<ActionCandidate> candidates, Cancellation cancellation) {
        public Request {
            wireId(schemaVersion); wireId(modelId); wireId(requestId); wireInteger(snapshotId); wireInteger(epoch);
            wireField(field); Objects.requireNonNull(source); Objects.requireNonNull(frame);
            Objects.requireNonNull(timestampDomain); Objects.requireNonNull(sync);
            wireInteger(issuedUs); wireInteger(historyCutoffUs); wireInteger(validUntilUs); wireInteger(horizonUs); wireInteger(stepUs);
            history=List.copyOf(history); targets=List.copyOf(targets); candidates=List.copyOf(candidates);
            if(history.isEmpty()||history.size()>MAX_HISTORY||targets.isEmpty()||targets.size()>MAX_TARGETS||candidates.isEmpty()||candidates.size()>MAX_CANDIDATES) throw new IllegalArgumentException("request capacity");
            Objects.requireNonNull(cancellation);
        }
    }
    /** Invalid-mask samples must use zero placeholders. Confidence is optional and cannot replace covariance. */
    record ForecastSample(long offsetUs, boolean validMask, double xM, double yM,
                          double vxMps, double vyMps, double covarianceXxM2,
                          double covarianceXyM2, double covarianceYyM2, Double confidence) {}
    record Forecast(String candidateId, String targetId, List<ForecastSample> samples) {
        public Forecast {
            wireId(candidateId); wireId(targetId); samples=List.copyOf(samples);
            if(samples.isEmpty()||samples.size()>MAX_SAMPLES) throw new IllegalArgumentException("forecast capacity");
        }
    }
    /** Not a track estimate, observed measurement, timed robot trajectory, or calibrated model by itself. */
    record Reply(String schemaVersion, String modelId, String requestId, long snapshotId,
                 long epoch, FieldIdentity field, SourceContext source, Frame frame,
                 TimestampDomain timestampDomain, long historyCutoffUs, long generatedUs,
                 long validUntilUs, long horizonUs, long stepUs, ForecastKind kind,
                 String uncertaintyCalibrationId, List<Forecast> forecasts) {
        public Reply {
            wireId(schemaVersion); wireId(modelId); wireId(requestId); wireInteger(snapshotId); wireInteger(epoch);
            wireField(field); Objects.requireNonNull(source); Objects.requireNonNull(frame);
            Objects.requireNonNull(timestampDomain); Objects.requireNonNull(kind);
            wireInteger(historyCutoffUs); wireInteger(generatedUs); wireInteger(validUntilUs); wireInteger(horizonUs); wireInteger(stepUs);
            Objects.requireNonNull(uncertaintyCalibrationId); if(uncertaintyCalibrationId.codePointCount(0,uncertaintyCalibrationId.length())>256) throw new IllegalArgumentException("calibration identity too long"); forecasts=List.copyOf(forecasts);
            if(forecasts.size()>MAX_CANDIDATES*MAX_TARGETS) throw new IllegalArgumentException("reply capacity");
        }
    }
}
