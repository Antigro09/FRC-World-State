package org.frcworldstate.vision;

import java.util.Objects;
import java.util.Optional;
import org.customvision.protocol.VisionClient;
import org.frcworldstate.core.WorldEngine;

/**
 * Small admitted-observation bridge; no I/O, polling, estimator, scheduler or motor access.
 * The robot owns capture-time ego history, resets/rebinding and source-health supervision.
 */
public final class VisionTrackBridge {
    /** Adapter and history/association outcomes are distinct; normalization alone is not insertion. */
    public record Decision(VisionClient.Measurement measurement, CustomVisionAdapter.Result normalization,
            Optional<WorldEngine.FrameDecision> tracking) {
        public Decision {
            Objects.requireNonNull(measurement); Objects.requireNonNull(normalization); Objects.requireNonNull(tracking);
            if (normalization.frame().isPresent() != tracking.isPresent())
                throw new IllegalArgumentException("accepted normalization requires its tracking decision");
        }
        public int insertedObservations() { return tracking.map(WorldEngine.FrameDecision::acceptedMeasurements).orElse(0); }
    }

    private final CustomVisionAdapter adapter;
    private final WorldEngine engine;
    private final VisionClient.DeliveryGate deliveryGate;
    public VisionTrackBridge(CustomVisionAdapter adapter, WorldEngine engine, VisionClient client) {
        this(adapter, engine, gate(client));
    }
    /** Use rig.observationDeliveryGate() for the facade's current camera mode/configuration fences. */
    public VisionTrackBridge(CustomVisionAdapter adapter, WorldEngine engine, VisionClient.DeliveryGate deliveryGate) {
        if (adapter == null) throw new IllegalArgumentException("Missing binding: configured vision geometry/time adapter");
        if (engine == null) throw new IllegalArgumentException("Missing binding: robot-owned world engine and capture pose history");
        if (deliveryGate == null) throw new IllegalArgumentException("Missing binding: originating rig/client delivery gate");
        if (adapter.binding().worldEpoch() != engine.epoch()) throw new IllegalArgumentException("adapter/world epoch differs; reset and rebuild binding explicitly");
        this.adapter = adapter; this.engine = engine; this.deliveryGate = deliveryGate;
    }

    /** CVJ supplies nanoseconds. Recheck source generation/liveness before one conversion into core us. */
    public Decision acceptRobotNanoseconds(VisionClient.Measurement measurement, long nowRobotNs) {
        CustomVisionAdapter.Result normalized = adapter.adaptDeliverable(deliveryGate, measurement, nowRobotNs);
        Optional<WorldEngine.FrameDecision> tracking = normalized.frame().map(frame ->
                engine.accept(frame, org.frcworldstate.core.TimeDomains.mappedRobotNsToUs(nowRobotNs)));
        return new Decision(measurement, normalized, tracking);
    }
    private static VisionClient.DeliveryGate gate(VisionClient client) {
        if (client == null) throw new IllegalArgumentException("Missing binding: originating consume-once vision client");
        return client::isDeliverable;
    }
}
