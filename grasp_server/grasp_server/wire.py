"""Binary wire helpers for request-response transport."""

from __future__ import annotations

import struct
from typing import Any

import msgpack
import numpy as np


def encode_packet(
    message_type: str,
    scalars: dict[str, Any] | None = None,
    arrays: dict[str, np.ndarray] | None = None,
    blobs: dict[str, bytes] | None = None,
) -> bytes:
    header: dict[str, Any] = {
        "type": message_type,
        "scalars": scalars or {},
        "arrays": [],
        "blobs": [],
    }
    body_parts: list[bytes] = []

    for name, value in (arrays or {}).items():
        array = np.ascontiguousarray(np.asarray(value))
        body = array.tobytes()
        header["arrays"].append(
            {
                "name": name,
                "shape": list(array.shape),
                "dtype": str(array.dtype),
                "nbytes": len(body),
            }
        )
        body_parts.append(body)

    for name, blob in (blobs or {}).items():
        blob_bytes = bytes(blob)
        header["blobs"].append({"name": name, "nbytes": len(blob_bytes)})
        body_parts.append(blob_bytes)

    header_bytes = msgpack.packb(header, use_bin_type=True)
    return struct.pack("<I", len(header_bytes)) + header_bytes + b"".join(body_parts)


def decode_packet(data: bytes) -> dict[str, Any]:
    header_len = struct.unpack("<I", data[:4])[0]
    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = memoryview(data)[4 + header_len :]
    offset = 0

    arrays: dict[str, np.ndarray] = {}
    for meta in header.get("arrays", []):
        dtype = np.dtype(meta["dtype"])
        nbytes = int(meta["nbytes"])
        shape = tuple(meta["shape"])
        array = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape).copy()
        arrays[meta["name"]] = array
        offset += nbytes

    blobs: dict[str, bytes] = {}
    for meta in header.get("blobs", []):
        nbytes = int(meta["nbytes"])
        blobs[meta["name"]] = bytes(body[offset : offset + nbytes])
        offset += nbytes

    return {
        "type": header["type"],
        "scalars": dict(header.get("scalars", {})),
        "arrays": arrays,
        "blobs": blobs,
    }
