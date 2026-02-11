import requests
import attr
from labgrid.driver.common import Driver
from labgrid.factory import target_factory
from labgrid.resource import NetworkService

@target_factory.reg_driver
@attr.s(eq=False)
class HttpLedDriver(Driver):
    bindings = {"svc": NetworkService}

    def on_activate(self):
        r = requests.get(f"http://{self.svc.address}:{self.svc.port}/on", timeout=2)
        r.raise_for_status()

    def on_deactivate(self):
        r = requests.get(f"http://{self.svc.address}:{self.svc.port}/off", timeout=2)
        r.raise_for_status()
