import os
import subprocess
import pytest
from labgrid import Environment
import http_led_driver  # registers driver
import probe_rs_driver  # registers driver

COORD = os.environ["LG_COORDINATOR"]
PLACE = "flatsat"

MATCH_LED   = "*/test-indicator-led/NetworkService/led"
MATCH_OPEN_LST_1 = "*/open-lst-1-probers/NetworkService/probe-open-lst-1"
Match_LOWER_SENSOR = "*/lower-sensor-probers/NetworkService/probe-lower-sensor"

def lg(*args, check=True):
    return subprocess.run(
        ["labgrid-client", "--coordinator", COORD, *args],
        text=True,
        capture_output=True,
        check=check,
    )

@pytest.fixture(scope="session", autouse=True)
def labgrid_session():
    lg("-p", PLACE, "delete", check=False)
    lg("-p", PLACE, "create", check=False)
    lg("-p", PLACE, "add-match", MATCH_LED, check=False)
    lg("-p", PLACE, "add-match", MATCH_OPEN_LST_1, check=False)
    lg("-p", PLACE, "add-match", Match_LOWER_SENSOR, check=False)
    print(lg("-p", PLACE, "show").stdout)

    # IMPORTANT: acquire before Environment() so RemotePlace can list resources
    lg("-p", PLACE, "acquire")

    try:
        env = Environment("env.yaml")
        t = env.get_target("main")
        t.get_driver("HttpLedDriver")  # LED ON via driver activation
        t.get_driver("ProbeRsDriver", name="probe-open-lst-1")
        t.get_driver("ProbeRsDriver", name="probe-lower-sensor")

        yield t
    finally:
        # Optional: ensure driver hooks run before releasing place
        try:
            t.get_driver("ProbeRsDriver", name="probe-open-lst-1").deactivate()
            t.get_driver("ProbeRsDriver", name="probe-lower-sensor").deactivate()
        except Exception:
            pass
        try:
            t.get_driver("HttpLedDriver").deactivate()
        except Exception:
            pass
        # release place (will also make LED OFF if your driver runs on deactivate;
        # otherwise call driver.deactivate() explicitly before releasing)
        lg("-p", PLACE, "release", check=False)
