import pytest
from redrhex_lowlevel_bridge.power_operation import plan_for,PowerFailure

@pytest.mark.parametrize('mask,plan',[(0,['digital','sensors','relay']),(1,['sensors','relay']),(3,['relay']),(7,[])])
def test_ensure_on_monotonic(mask,plan):
    assert plan_for('ensure-on',mask)==plan
    assert plan_for('relay',mask)==plan
    assert plan_for('sequence',mask,True)==plan

@pytest.mark.parametrize('mask,plan',[(0,['digital','sensors']),(1,['sensors']),(3,[]),(7,[])])
def test_legacy_sequence_never_drops_signal_or_relay(mask,plan):
    assert plan_for('sequence',mask)==plan

@pytest.mark.parametrize('mask',[-1,2,4,5,6])
def test_invalid_feedback_does_not_authorize_on(mask):
    with pytest.raises(PowerFailure):plan_for('ensure-on',mask)
