import os
import subprocess
import pytest
from labgrid import Environment
import drivers.http_led_driver  # registers driver
import drivers.probe_rs_driver  # registers driver

COORD = os.environ["LG_COORDINATOR"]
PLACE = "flatsat"

MATCH_LED   = "*/test-indicator-led/NetworkService/led"
MATCH_EPS = "*/eps-probers/NetworkService/probe-eps"

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
    lg("-p", PLACE, "add-match", MATCH_EPS, check=False)
    print(lg("-p", PLACE, "show").stdout)

    # IMPORTANT: acquire before Environment() so RemotePlace can list resources
    lg("-p", PLACE, "acquire")

    try:
        env = Environment("env.yaml")
        t = env.get_target("main")
        t.get_driver("HttpLedDriver")  # LED ON via driver activation
        t.get_driver("ProbeRsDriver", name="probe-eps")

        yield t
    finally:
        # Optional: ensure driver hooks run before releasing place
        try:
            t.get_driver("ProbeRsDriver", name="probe-eps").deactivate()
        except Exception:
            pass
        try:
            t.get_driver("HttpLedDriver").deactivate()
        except Exception:
            pass
        # release place (will also make LED OFF if your driver runs on deactivate;
        # otherwise call driver.deactivate() explicitly before releasing)
        lg("-p", PLACE, "release", check=False)
