"""One pinned Aorta group limit shared with the C++ endpoint precheck."""
from pathlib import Path
import re
import secrets

# Aorta's private MAX_GROUP_LEN is mirrored once, in aorta_contract.h.
_declaration = re.search(r"MAX_GROUP_LEN\s*=\s*(\d+)\s*;",
                         Path(__file__).with_name("aorta_contract.h").read_text())
if _declaration is None:
    raise RuntimeError("missing pinned Aorta MAX_GROUP_LEN declaration")
MAX_GROUP_LEN = int(_declaration[1])
GROUP_PREFIX = "mock-sdkmain-"


def validate_group(group):
    if isinstance(group, str) and len(group.encode("utf-8")) > MAX_GROUP_LEN:
        raise ValueError(f"Aorta group exceeds max length {MAX_GROUP_LEN} (utf-8 bytes): "
                         f"got {len(group.encode('utf-8'))}")
    if not isinstance(group, str) or not group.startswith(GROUP_PREFIX) or any(
            not (c.isascii() and (c.isalnum() or c in "-_")) for c in group):
        raise ValueError("group must be a unique mock-sdkmain-* name; default/production groups are forbidden")
    return group


def new_group():
    # At the pinned limit this preserves112 random bits in a19-character
    # URL-safe suffix; unlike uuid32 the complete group fits Aorta's limit.
    suffix_bytes = (MAX_GROUP_LEN - len(GROUP_PREFIX)) * 3 // 4
    return validate_group(GROUP_PREFIX + secrets.token_urlsafe(suffix_bytes))
