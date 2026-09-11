"""Only localhost/domain232; native config fixture, production CLI implementation."""
import os
from contextlib import contextmanager
if os.environ.get('ROS_DOMAIN_ID')!='232' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit(90)
from redrhex_lowlevel_bridge import rinbo_power_tool as tool
@contextmanager
def configuration():
    yield tool.LegConfiguration(tool.ORIN_LEGS_CONFIG_PATH,1,15,'fixture-r15',('L3',),('L1','L2','R1','R2','R3'))
tool._pinned_leg_configuration=configuration
tool.main()
