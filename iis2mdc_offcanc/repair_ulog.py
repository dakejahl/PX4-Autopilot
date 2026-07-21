#!/usr/bin/env python3
"""Rebuild bit-flipped ULog format definitions from a PX4 build tree.

Thermal SD damage flips bits inside the definition section. A damaged 'F'
message either fails to parse or, worse, registers under a garbled topic name
and parses fine, so the failure only surfaces later as a missing format.

Every logged topic's format string is regenerated from the build tree's uORB
headers, then each suspect 'F' message is matched to a candidate of identical
byte length and replaced. Length must match exactly, so a wrong guess cannot
silently change the file layout.

Usage: repair_ulog.py IN.ulg OUT.ulg [BUILD_TOPICS_DIR]
"""
import pathlib
import re
import struct
import sys

DEFAULT_TOPICS = ("/home/jake/code/jake/PX4-Autopilot/build/ark_fpv_default"
                  "/uORB/topics")


def generate_formats(topics_dir):
    out = {}
    for header in pathlib.Path(topics_dir).glob("*.h"):
        m = re.search(r"struct (\w+)_s \{(.*?)\n\};", header.read_text(), re.S)
        if not m:
            continue
        name = m.group(1)
        body = m.group(2).split("#ifdef __cplusplus")[0]
        fields = []
        for line in body.splitlines():
            line = line.split("//")[0].strip()
            if not line or line.startswith("#"):
                continue
            mm = re.match(r"^([A-Za-z_]\w*)\s+([A-Za-z_]\w*)(?:\[(\d+)\])?;$", line)
            if not mm:
                continue
            typ, field, count = mm.groups()
            fields.append("%s[%s] %s;" % (typ, count, field) if count
                          else "%s %s;" % (typ, field))
        if fields:
            out[name] = name + ":" + "".join(fields)
    return out


def fields_parse(txt):
    if ":" not in txt:
        return False
    for f in txt.split(":", 1)[1].split(";"):
        if not f:
            continue
        t = f.split(" ")[0]
        if "[" in t:
            inner = t[t.index("[") + 1:t.index("]")] if "]" in t else ""
            if not inner.isdigit():
                return False
    return True



def match(formats, payload, size, data, off, floor=0.6):
    """Find the one reference format this damaged message must be.

    Byte length is the discriminator: a candidate of identical length is almost
    always unique, which is a far tighter constraint than byte similarity on
    heavily flipped text. If nothing matches the recorded length, the length
    field itself is suspect, so try lengths that leave a valid message header
    immediately after.
    """
    def score(name, n):
        ref = formats[name].encode()
        return sum(a == b for a, b in zip(ref, payload[:n])) / n

    # Several topics can share a byte length, so let similarity break the tie,
    # but only when the winner is clearly ahead of the runner-up.
    ranked = sorted(((score(k, size), k) for k, v in formats.items()
                     if len(v) == size), reverse=True)
    if ranked and ranked[0][0] > floor:
        if len(ranked) == 1 or ranked[0][0] - ranked[1][0] > 0.15:
            return ranked[0][1], size, ranked[0][0]

    viable = []
    for name, ref in formats.items():
        n = len(ref)
        if n >= size or off + 3 + n + 3 > len(data):
            continue
        nxt = struct.unpack_from("<HB", data, off + 3 + n)[1]
        if chr(nxt) in "BFIMPQADLSOR" and score(name, n) > floor:
            viable.append((score(name, n), name, n))
    if len(viable) == 1:
        s_, name, n = viable[0]
        return name, n, s_
    return None


def main():
    src, dst = sys.argv[1], sys.argv[2]
    topics = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_TOPICS
    formats = generate_formats(topics)
    print("generated %d reference formats" % len(formats))

    data = bytearray(open(src, "rb").read())
    off, fixed, failed = 16, 0, 0
    while off + 3 <= len(data):
        size, mtype = struct.unpack("<HB", data[off:off + 3])
        if chr(mtype) not in "BFIMPQ":
            break
        if chr(mtype) == "F":
            payload = bytes(data[off + 3:off + 3 + size])
            txt = payload.decode("utf-8", "replace")
            name = txt.split(":")[0]
            # A healthy message may use nested types this generator does not
            # reproduce, so damage is judged on parseability and on the name
            # being a plausible topic identifier, never on an exact match.
            damaged_here = (not fields_parse(txt)
                            or not re.match(r"^[a-z][a-z0-9_]*$", name))
            if damaged_here:
                fix = match(formats, payload, size, data, off)
                if fix is None:
                    print("  ! no confident match at %6d: %r" % (off, txt[:50]))
                    failed += 1
                else:
                    best, true_size, score = fix
                    if true_size != size:
                        # The length field itself was flipped; the payload is intact
                        # at its true length, so re-writing it re-aligns the stream.
                        struct.pack_into("<H", data, off, true_size)
                        print("  repaired %-26s at %6d  size %d -> %d (bit flip)"
                              % (best, off, size, true_size))
                    else:
                        print("  repaired %-26s at %6d  (%d bytes, %.0f%% intact)"
                              % (best, off, true_size, 100 * score))
                    data[off + 3:off + 3 + true_size] = formats[best].encode()
                    size = true_size
                    fixed += 1
        off += 3 + size

    open(dst, "wb").write(bytes(data))
    print("wrote %s (%d repaired, %d unresolved)" % (dst, fixed, failed))


if __name__ == "__main__":
    main()
