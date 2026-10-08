package org.frcworldstate.core;

/** Version-specific metadata conversions. JSON *_us is NEVER passed through an NT metadata converter. */
public final class TimeDomains {
    private TimeDomains() {}
    public static long nt2026MetadataUs(long microseconds) { return Geometry.time(microseconds); }
    /** Floor rounds a mapped nonnegative capture instant backwards by <1us, never forward. */
    public static long nt2027Alpha7MetadataToUs(long nanoseconds) { return Geometry.time(nanoseconds)/1000; }
    public static long mappedRobotNsToUs(long nanoseconds) { return Geometry.time(nanoseconds)/1000; }
    public static long jsonUsToNs(long microseconds) { return Math.multiplyExact(Geometry.time(microseconds),1000); }
}
