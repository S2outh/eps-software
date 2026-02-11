import os


def test_flash(labgrid_session):
    t = labgrid_session
    d = t.get_driver("ProbeRsDriver", name="probe-eps")

    binary = "fw/eps-software"          # path if not in repo root

    fw_version = os.environ.get("FW_VERSION") or "no-version"
    fw_hash = os.environ.get("FW_HASH") or "no-hash"

    print(f"flashing eps with firmware {fw_version}")
    # flash binary
    d.flash(binary)
    print("finished flashing eps")
    print("verifying version")

    # attach with reset and read logs line-by-line from the test
    stream = d.attach_with_reset(binary, timeout=30.0)

    try:
        it = stream.iter_lines()

        try:
            first_line = next(it)
        except StopIteration:
            raise AssertionError("no output received from probe-rs")

        assert f"Launching: FW version={fw_version} hash={fw_hash}" in first_line, f"unexpected first line: {first_line}"

    finally:
        # Always stop the subprocess, even on assertion failure
        stream.close()
