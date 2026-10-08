package org.frcworldstate.vision;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import org.customvision.protocol.ClockMapper;
import org.customvision.protocol.Packet;
import org.customvision.protocol.SourceKey;
import org.customvision.protocol.SourceSession;
import org.customvision.protocol.TimeVersion;
import org.customvision.protocol.VisionClient;
import org.frcworldstate.core.Geometry;
import org.frcworldstate.core.TimeDomains;
import org.frcworldstate.core.World;

/** Bounded normalization of already-decoded, session-approved object observations. No field projection or I/O. */
public final class CustomVisionAdapter {
    public static final String PROFILE = "custom-vision-schema2-2026.1";
    private static final String FRAME = "robot_relative_at_capture_wpilib_nwu";
    private static final String METHOD = "calibrated_ray_target_height_plane";

    /** Rebuild this binding after an authorized source/boot, connection, geometry or world epoch change. */
    public record Binding(SourceKey source, String profile, String activeBootId, String robotEpoch,
            long worldEpoch, long connectionEpoch, TimeVersion timeVersion,
            String calibrationRevision, String mountRevision, Optional<String> fieldLayoutRevision,
            String correlationGroup, String geometryProfileRevision) {
        public Binding {
            Objects.requireNonNull(source); Objects.requireNonNull(timeVersion);
            Objects.requireNonNull(fieldLayoutRevision);
            Geometry.id(profile); Geometry.id(activeBootId); Geometry.id(robotEpoch);
            Geometry.time(worldEpoch); Geometry.time(connectionEpoch);
            revision(calibrationRevision); revision(mountRevision); fieldLayoutRevision.ifPresent(CustomVisionAdapter::revision);
            Geometry.id(correlationGroup); Geometry.id(geometryProfileRevision);
            if (!PROFILE.equals(profile) || !"object".equals(source.type()))
                throw new IllegalArgumentException("adapter requires the named object profile");
        }
    }

    /** Configuration assumptions are explicit, measured later, and are not a hardware safety qualification. */
    public record Policy(double targetHeightM, double heightToleranceM, double minConfidence,
            double maxRangeM, double varianceFloorM2, double maxReportedVarianceM2,
            double wireCovarianceToleranceM2, long maxClockUncertaintyNs,
            long maxCaptureAgeUs, long maxPublicationAgeUs, int maxTargets) {
        public Policy {
            Geometry.finite(targetHeightM); Geometry.nonnegative(heightToleranceM);
            Geometry.nonnegative(minConfidence); Geometry.nonnegative(maxRangeM);
            Geometry.nonnegative(varianceFloorM2); Geometry.nonnegative(maxReportedVarianceM2);
            Geometry.nonnegative(wireCovarianceToleranceM2); Geometry.time(maxClockUncertaintyNs);
            Geometry.time(maxCaptureAgeUs); Geometry.time(maxPublicationAgeUs);
            if (minConfidence > 1 || maxRangeM == 0 || varianceFloorM2 == 0
                    || maxReportedVarianceM2 == 0 || wireCovarianceToleranceM2 > maxReportedVarianceM2
                    || maxTargets < 1 || maxTargets > 256)
                throw new IllegalArgumentException("invalid explicit adapter bounds");
        }
    }

    public enum Status { ACCEPTED, UNAVAILABLE, REJECTED }
    public enum Reason { ACCEPTED, SOURCE_MISMATCH, PROFILE_MISMATCH, BOOT_MISMATCH,
        DISCONNECTED, MISSING_FAMILY, INVALID_FAMILY, EMPTY_OBSERVATIONS, REVISION_MISMATCH,
        NO_NEW_MEASUREMENT, LIFECYCLE_MISMATCH, CLOCK_REJECTED, CLOCK_MISMATCH, FUTURE, STALE, TIME_OVERFLOW, INVALID_GEOMETRY,
        UNSUPPORTED_ESTIMATE, TARGET_PLANE_MISMATCH, INVALID_QUALITY, INVALID_UNCERTAINTY, CAPACITY }

    /** Complete immutable decoder/clock provenance stays separate from the normalized core measurement. */
    public record Provenance(Packet rawPacket, SourceSession.Result lifecycleAdmission, ClockMapper.Result clockMapping,
            String geometryProfileRevision, String correlationGroup,
            Optional<String> observedFieldLayoutRevision) {}
    public record Result(Status status, Reason reason, String detail,
            Optional<World.ObservationFrame> frame, Provenance provenance) {
        public Result {
            Objects.requireNonNull(status); Objects.requireNonNull(reason); Objects.requireNonNull(detail);
            Objects.requireNonNull(frame); Objects.requireNonNull(provenance);
            if ((status == Status.ACCEPTED) != frame.isPresent())
                throw new IllegalArgumentException("only accepted results contain a normalized measurement");
        }
        /** Unavailable/rejected observations never establish free space. */
        public boolean provesFreeSpace() { return false; }
    }

    private static final class Gate extends RuntimeException {
        private static final long serialVersionUID = 1L;
        final Reason reason;
        Gate(Reason reason, String detail) { super(detail); this.reason = reason; }
    }
    private final Binding binding;
    private final Policy policy;
    public CustomVisionAdapter(Binding binding, Policy policy) {
        this.binding = Objects.requireNonNull(binding); this.policy = Objects.requireNonNull(policy);
    }
    public Binding binding() { return binding; }
    public Policy policy() { return policy; }

    /** Normalize historical admission provenance. The caller must recheck current client deliverability. */
    public Result adapt(VisionClient.Measurement measurement, long nowUs) {
        Objects.requireNonNull(measurement, "drained admitted measurement required");
        return adapt(measurement.packet(), measurement.admission(), measurement.clock(), nowUs);
    }

    /** Live callback/manual-drain ingress; use the original nanosecond clock without a lossy roundtrip. */
    public Result adaptDeliverable(VisionClient client, VisionClient.Measurement measurement, long nowRobotNs) {
        Objects.requireNonNull(client, "originating client required");
        return adaptDeliverable(client::isDeliverable, measurement, nowRobotNs);
    }

    /** Facade gate also includes its camera mode and binding fences; it exposes no raw client. */
    public Result adaptDeliverable(VisionClient.DeliveryGate gate, VisionClient.Measurement measurement, long nowRobotNs) {
        Objects.requireNonNull(gate, "originating rig/client delivery gate required"); Objects.requireNonNull(measurement);
        Geometry.time(nowRobotNs);
        if (!gate.isDeliverable(measurement, nowRobotNs)) {
            Packet packet = measurement.packet();
            Provenance provenance = new Provenance(packet, measurement.admission(), measurement.clock(),
                    binding.geometryProfileRevision(), binding.correlationGroup(), optionalString(packet.fields().get("field_layout_revision")));
            return unavailable(Reason.NO_NEW_MEASUREMENT,
                    "historically admitted envelope is no longer deliverable by its originating rig/client", provenance);
        }
        return adapt(measurement, TimeDomains.mappedRobotNsToUs(nowRobotNs));
    }

    /**
     * Invoke only for a publication approved by CVJ's SourceSession/liveness policy. Invalidation is
     * handled before capture dedup there. This pure adapter does not authorize a retained old boot.
     */
    public Result adapt(Packet packet, SourceSession.Result admission, ClockMapper.Result mapping, long nowUs) {
        Objects.requireNonNull(packet); Objects.requireNonNull(admission); Objects.requireNonNull(mapping); Geometry.time(nowUs);
        Provenance provenance = new Provenance(packet, admission, mapping, binding.geometryProfileRevision(),
                binding.correlationGroup(), optionalString(packet.fields().get("field_layout_revision")));
        if (!binding.source().equals(packet.source()))
            return rejected(Reason.SOURCE_MISMATCH, "complete configured source differs", provenance);
        if (packet.schemaVersion() != 2 || !binding.profile().equals(packet.profile()) || packet.packetSeq().isEmpty())
            return rejected(Reason.PROFILE_MISMATCH, "named schema-2 profile and publication identity required", provenance);
        if (!binding.activeBootId().equals(packet.bootId()))
            return rejected(Reason.BOOT_MISMATCH, "publication is not from the configured active boot", provenance);
        if (!packet.connected()) return unavailable(Reason.DISCONNECTED, "camera unavailable", provenance);
        Optional<Map<String, Object>> family = packet.family("objects");
        if (family.isEmpty()) return unavailable(Reason.MISSING_FAMILY, "object family absent", provenance);
        Map<String, Object> objects = family.orElseThrow();
        if (!Boolean.TRUE.equals(objects.get("valid")))
            return unavailable(Reason.INVALID_FAMILY, "object family failed producer gates", provenance);
        if (admission.kind() != SourceSession.Kind.ACCEPTED || admission.newlyAcceptedMeasurement().isEmpty())
            return unavailable(Reason.NO_NEW_MEASUREMENT, "lifecycle did not emit a new reusable capture observation", provenance);
        if (!admission.newlyAcceptedMeasurement().orElseThrow().equals(packet))
            return rejected(Reason.LIFECYCLE_MISMATCH, "lifecycle admission belongs to a different coherent packet", provenance);
        if (!binding.calibrationRevision().equals(packet.fields().get("calibration_revision"))
                || !binding.mountRevision().equals(packet.fields().get("mount_revision"))
                || binding.fieldLayoutRevision().isPresent()
                    && !binding.fieldLayoutRevision().orElseThrow().equals(packet.fields().get("field_layout_revision")))
            return rejected(Reason.REVISION_MISMATCH, "missing or changed configured geometry revision", provenance);
        if (mapping.capture().isEmpty())
            return rejected(Reason.CLOCK_REJECTED, mapping.rejection().orElseThrow().reason().name(), provenance);
        try {
            ClockMapper.MappedCapture capture = mapping.capture().orElseThrow();
            validateClock(packet, capture);
            long captureUs = TimeDomains.mappedRobotNsToUs(capture.robotCaptureNs());
            // ntTimestampNs is already converted by the selected CVJ metadata adapter. JSON stays us.
            long publicationNs = Math.addExact(capture.ntTimestampNs(), capture.localToRobotOffsetNs());
            long publicationUs = TimeDomains.mappedRobotNsToUs(publicationNs);
            if (captureUs > nowUs || publicationUs > nowUs)
                throw new Gate(Reason.FUTURE, "capture or publication is after current robot time");
            if (publicationNs < capture.robotCaptureNs())
                throw new Gate(Reason.CLOCK_MISMATCH, "publication precedes capture");
            if (nowUs - captureUs > policy.maxCaptureAgeUs()
                    || nowUs - publicationUs > policy.maxPublicationAgeUs())
                throw new Gate(Reason.STALE, "capture or publication exceeds configured age");
            if (!FRAME.equals(objects.get("frame"))) throw new Gate(Reason.INVALID_GEOMETRY, "unsupported object frame");
            if (!Boolean.FALSE.equals(objects.get("motion_compensated")))
                throw new Gate(Reason.UNSUPPORTED_ESTIMATE, "motion-compensated object estimates are not observations");
            List<?> targets = list(objects.get("targets"), Reason.INVALID_GEOMETRY, "canonical targets missing");
            if (targets.size() > policy.maxTargets()) throw new Gate(Reason.CAPACITY, "configured target capacity exceeded");
            if (targets.isEmpty()) return unavailable(Reason.EMPTY_OBSERVATIONS, "no object observations", provenance);
            List<World.Observation> observations = new ArrayList<>(targets.size());
            Set<Long> localIds = new HashSet<>();
            for (Object raw : targets) {
                Map<?, ?> target = object(raw, Reason.INVALID_GEOMETRY, "target object missing");
                if (!Boolean.TRUE.equals(target.get("valid")) || !FRAME.equals(target.get("frame"))
                        || !METHOD.equals(target.get("method")) || !Boolean.TRUE.equals(target.get("approximate")))
                    throw new Gate(Reason.INVALID_GEOMETRY, "target is not accepted calibrated ray-plane geometry");
                if (!Boolean.TRUE.equals(target.get("observed")) || !Boolean.FALSE.equals(target.get("predicted")))
                    throw new Gate(Reason.UNSUPPORTED_ESTIMATE, "only current observations are supported");
                long trackId = integer(target.get("track_id"), "local track id");
                if (trackId < 1 || !localIds.add(trackId)) throw new Gate(Reason.INVALID_GEOMETRY, "duplicate/invalid local target identity");
                if (!Objects.equals(target.get("capture_monotonic_us"), packet.fields().get("capture_monotonic_us")))
                    throw new Gate(Reason.INVALID_GEOMETRY, "target belongs to another host capture");
                double height = number(target.get("target_height_m"), Reason.INVALID_GEOMETRY, "target plane missing");
                if (Math.abs(height - policy.targetHeightM()) > policy.heightToleranceM())
                    throw new Gate(Reason.TARGET_PLANE_MISMATCH, "target plane differs from configured assumption");
                List<?> translation = list(target.get("translation_m"), Reason.INVALID_GEOMETRY, "robot-relative translation missing");
                if (translation.size() != 3) throw new Gate(Reason.INVALID_GEOMETRY, "translation must have 3 SI components");
                double x = number(translation.get(0), Reason.INVALID_GEOMETRY, "invalid x");
                double y = number(translation.get(1), Reason.INVALID_GEOMETRY, "invalid y");
                double z = number(translation.get(2), Reason.INVALID_GEOMETRY, "invalid z");
                if (Math.abs(z - height) > policy.heightToleranceM() || Math.hypot(x, y) > policy.maxRangeM())
                    throw new Gate(Reason.INVALID_GEOMETRY, "translation outside configured target plane/range");
                String label = string(target.get("label"), Reason.INVALID_QUALITY, "object class label unavailable");
                double confidence = number(target.get("confidence"), Reason.INVALID_QUALITY, "normalized confidence unavailable");
                if (confidence < policy.minConfidence() || confidence > 1)
                    throw new Gate(Reason.INVALID_QUALITY, "confidence outside configured supported interval");
                Geometry.Uncertainty uncertainty = uncertainty(target.get("uncertainty"));
                // A source-local association ID is diagnostic identity, not a permanent physical track ID.
                observations.add(new World.Observation("frame:" + packet.frameId() + "/target:" + trackId,
                        label, confidence, new Geometry.Vec2(x, y), uncertainty));
            }
            World.SourceStamp stamp = new World.SourceStamp(sourceIdentity(packet.source(), packet.profile()),
                    packet.bootId(), binding.calibrationRevision(), binding.mountRevision(), binding.correlationGroup(),
                    packet.frameId(), captureUs, publicationUs, policy.targetHeightM());
            return new Result(Status.ACCEPTED, Reason.ACCEPTED, "current capture observations normalized",
                    Optional.of(new World.ObservationFrame(binding.worldEpoch(), stamp, observations)), provenance);
        } catch (Gate gate) {
            return rejected(gate.reason, gate.getMessage(), provenance);
        } catch (ArithmeticException exception) {
            return rejected(Reason.TIME_OVERFLOW, "checked clock arithmetic overflow", provenance);
        } catch (IllegalArgumentException exception) {
            return rejected(Reason.INVALID_GEOMETRY, exception.getMessage(), provenance);
        }
    }

    private void validateClock(Packet packet, ClockMapper.MappedCapture c) {
        if (!binding.robotEpoch().equals(c.robotEpoch()) || binding.connectionEpoch() != c.connectionEpoch()
                || binding.timeVersion() != c.metadataVersion() || c.rawCaptureServerUs() != packet.captureServerUs()
                || !packet.timeSyncValid() || !c.producerCorrectionAlreadyApplied()
                || c.epochVerification() == null || c.epochVerification().isBlank()
                || c.captureCorrectionEvidence() == null || c.captureCorrectionEvidence().isBlank()
                || c.robotCaptureNs() < 0 || c.ntTimestampNs() < 0 || c.ntServerTimeNs() < 0
                || c.synchronizationUncertaintyNs() < 0 || c.captureCorrectionUncertaintyNs() < 0
                || c.totalUncertaintyNs() < 0 || c.totalUncertaintyNs() > policy.maxClockUncertaintyNs())
            throw new Gate(Reason.CLOCK_MISMATCH, "mapping identity, correction evidence or uncertainty mismatch");
        Map<String, Object> timing = packet.family("timing").orElseThrow(
                () -> new Gate(Reason.CLOCK_MISMATCH, "named profile capture timing evidence missing"));
        if (!"nt_server".equals(timing.get("clock_domain")) || !"us".equals(timing.get("timestamp_unit"))
                || !"host_frame_read_complete".equals(timing.get("capture_event"))
                || !Boolean.TRUE.equals(timing.get("capture_correction_verified")))
            throw new Gate(Reason.CLOCK_MISMATCH, "mapping belongs to unverified/different capture provenance");
        double correctionMs = number(timing.get("capture_correction_uncertainty_ms"),
                Reason.CLOCK_MISMATCH, "capture correction uncertainty missing");
        if (correctionMs < 0 || BigDecimal.valueOf(correctionMs).multiply(BigDecimal.valueOf(1_000_000L)).longValueExact()
                != c.captureCorrectionUncertaintyNs())
            throw new Gate(Reason.CLOCK_MISMATCH, "raw correction uncertainty differs from mapping");
        if (Math.addExact(c.synchronizationUncertaintyNs(), c.captureCorrectionUncertaintyNs()) != c.totalUncertaintyNs()
                || c.metadataVersion().metadataToNanoseconds(c.rawNtTimestamp()) != c.ntTimestampNs()
                || c.metadataVersion().metadataToNanoseconds(c.rawNtServerTime()) != c.ntServerTimeNs()
                || c.metadataVersion().metadataToNanoseconds(c.rawServerMinusLocal()) != c.serverMinusLocalNs()
                || Math.addExact(Math.subtractExact(TimeDomains.jsonUsToNs(c.rawCaptureServerUs()),
                        c.serverMinusLocalNs()), c.localToRobotOffsetNs()) != c.robotCaptureNs())
            throw new Gate(Reason.CLOCK_MISMATCH, "mapped values do not match their pinned raw units");
    }

    /** Isotropic bound dominates the reported XY covariance; the configured floor is not another measurement. */
    private Geometry.Uncertainty uncertainty(Object raw) {
        Map<?, ?> value = object(raw, Reason.INVALID_UNCERTAINTY, "configured first-order uncertainty missing");
        if (!"first_order_configured_pixel_height_pitch_mount".equals(value.get("model")))
            throw new Gate(Reason.INVALID_UNCERTAINTY, "unsupported uncertainty model");
        List<?> rows = list(value.get("covariance_xy_m2"), Reason.INVALID_UNCERTAINTY, "covariance missing");
        if (rows.size() != 2) throw new Gate(Reason.INVALID_UNCERTAINTY, "covariance requires 2 rows");
        List<?> a = list(rows.get(0), Reason.INVALID_UNCERTAINTY, "covariance row missing");
        List<?> b = list(rows.get(1), Reason.INVALID_UNCERTAINTY, "covariance row missing");
        if (a.size() != 2 || b.size() != 2) throw new Gate(Reason.INVALID_UNCERTAINTY, "covariance requires 2 columns");
        double xx = number(a.get(0), Reason.INVALID_UNCERTAINTY, "invalid variance");
        double xy = number(a.get(1), Reason.INVALID_UNCERTAINTY, "invalid covariance");
        double yx = number(b.get(0), Reason.INVALID_UNCERTAINTY, "invalid covariance");
        double yy = number(b.get(1), Reason.INVALID_UNCERTAINTY, "invalid variance");
        if (xx < 0 || yy < 0 || Math.abs(xy - yx) > policy.wireCovarianceToleranceM2())
            throw new Gate(Reason.INVALID_UNCERTAINTY, "negative or asymmetric covariance");
        double off = (xy + yx) / 2;
        double discriminant = Math.hypot(xx - yy, 2 * off);
        double smallest = (xx + yy - discriminant) / 2;
        double largest = (xx + yy + discriminant) / 2;
        if (!Double.isFinite(largest) || smallest < -policy.wireCovarianceToleranceM2()
                || largest > policy.maxReportedVarianceM2())
            throw new Gate(Reason.INVALID_UNCERTAINTY, "covariance outside configured PSD/quality bounds");
        // Explicit wire-rounding budget ensures the retained bound covers tolerated numeric drift.
        double variance = Math.max(policy.varianceFloorM2(), largest + policy.wireCovarianceToleranceM2());
        if (!Double.isFinite(variance)) throw new Gate(Reason.INVALID_UNCERTAINTY, "uncertainty overflow");
        return new Geometry.Uncertainty(variance, 0, variance);
    }

    /** Length framing prevents collisions even if a permitted source name contains separator characters. */
    public static String sourceIdentity(SourceKey source, String profile) {
        return "custom-vision:" + framed(source.namespace()) + framed(source.pipeline())
                + framed(source.type()) + framed(profile);
    }
    private static String framed(String value) { return value.length() + ":" + value; }
    private static String revision(String value) {
        if (value == null || !value.matches("sha256:[0-9a-f]{64}"))
            throw new IllegalArgumentException("configured revision requires sha256 lowercase hex");
        return value;
    }
    private static Optional<String> optionalString(Object raw) {
        return raw instanceof String s ? Optional.of(s) : Optional.empty();
    }
    private static String string(Object raw, Reason reason, String detail) {
        if (!(raw instanceof String s) || s.isBlank()) throw new Gate(reason, detail);
        return s;
    }
    private static long integer(Object raw, String detail) {
        if (!(raw instanceof Long value) || value < 0) throw new Gate(Reason.INVALID_GEOMETRY, detail);
        return value;
    }
    private static double number(Object raw, Reason reason, String detail) {
        if (!(raw instanceof Number value) || !Double.isFinite(value.doubleValue())) throw new Gate(reason, detail);
        return value.doubleValue();
    }
    private static List<?> list(Object raw, Reason reason, String detail) {
        if (!(raw instanceof List<?> value)) throw new Gate(reason, detail);
        return value;
    }
    private static Map<?, ?> object(Object raw, Reason reason, String detail) {
        if (!(raw instanceof Map<?, ?> value)) throw new Gate(reason, detail);
        return value;
    }
    private static Result rejected(Reason reason, String detail, Provenance provenance) {
        return new Result(Status.REJECTED, reason, detail, Optional.empty(), provenance);
    }
    private static Result unavailable(Reason reason, String detail, Provenance provenance) {
        return new Result(Status.UNAVAILABLE, reason, detail, Optional.empty(), provenance);
    }
}
