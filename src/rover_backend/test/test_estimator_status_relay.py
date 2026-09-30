"""estimator_status_relay forwards only ESTIMATOR_STATUS (#230)."""

from types import SimpleNamespace

from rover_backend.estimator_status_relay import EstimatorStatusRelay


def test_relay_forwards_only_estimator_status():
    published = []
    relay = object.__new__(EstimatorStatusRelay)
    relay._publisher = SimpleNamespace(publish=published.append)

    for msgid in (30, 33, 105, 0, 230, 147, 230):
        relay._source_callback(SimpleNamespace(msgid=msgid))

    assert [m.msgid for m in published] == [230, 230]
