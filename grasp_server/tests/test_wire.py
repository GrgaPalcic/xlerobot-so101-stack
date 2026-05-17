import numpy as np

from grasp_server.wire import decode_packet, encode_packet


def test_packet_round_trip():
    payload = encode_packet(
        "demo",
        scalars={"prompt": "red cube", "top_k": 5},
        arrays={"cloud": np.arange(12, dtype=np.float32).reshape(4, 3)},
        blobs={"jpeg": b"abc123"},
    )
    decoded = decode_packet(payload)
    assert decoded["type"] == "demo"
    assert decoded["scalars"]["prompt"] == "red cube"
    np.testing.assert_allclose(decoded["arrays"]["cloud"], np.arange(12, dtype=np.float32).reshape(4, 3))
    assert decoded["blobs"]["jpeg"] == b"abc123"
