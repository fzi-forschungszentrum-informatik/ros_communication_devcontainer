#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.                                                
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2026-01-09
#
# ---------------------------------------------------------------------

from __future__ import annotations

"""
Shared helpers to transform arbitrary ROS2 Python messages in-place.

We intentionally keep this module free of message-type imports so it works for
any generated ROS2 Python message (including nested messages and arrays).

Currently supported transformations:
- Frame ID remapping: fields named 'frame_id' and 'child_frame_id'
- Timestamp rebasing: fields named 'stamp' (e.g. builtin_interfaces/msg/Time),
  including header.stamp via recursion into header.
"""

from typing import Any, List, Tuple


# ======================================================================
#  Message structure helpers
# ======================================================================


def _is_sequence(value: Any) -> bool:
    return isinstance(value, (list, tuple))


def _is_ros_msg(obj: Any) -> bool:
    # Most generated ROS2 Python message classes provide this method.
    return hasattr(obj, "get_fields_and_field_types")


def _iter_field_names(obj: Any) -> List[str]:
    """
    Use ROS message public field names (e.g. 'header', 'transforms', 'markers'),
    not __slots__ (which are often private like '_header', '_transforms').
    """
    if _is_ros_msg(obj):
        try:
            return list(obj.get_fields_and_field_types().keys())
        except Exception:
            pass

    # Fallback: derive from __slots__ by stripping leading '_' (best-effort)
    if hasattr(obj, "__slots__"):
        out: List[str] = []
        for s in getattr(obj, "__slots__", []):
            if s == "_check_fields":
                continue
            if isinstance(s, str) and s.startswith("_"):
                out.append(s[1:])
            else:
                out.append(s)
        return out

    return []


def _is_time_msg(obj: Any) -> bool:
    # builtin_interfaces/msg/Time has fields: sec (int32), nanosec (uint32)
    return hasattr(obj, "sec") and hasattr(obj, "nanosec")


def _format_time(obj: Any) -> str:
    try:
        if _is_time_msg(obj):
            return f"{int(obj.sec)}.{int(obj.nanosec):09d}"
    except Exception:
        pass
    return str(obj)


# ======================================================================
#  Generic walker
# ======================================================================


def walk_ros_message(obj: Any, *, visit_field, path: str = "") -> None:
    """
    Depth-first traversal over a ROS2 Python message.

    `visit_field(obj, field_name, val, path)` is called for each field before recursion.
    - obj: the owning message object
    - field_name: the public field name
    - val: current field value
    - path: a dotted path prefix, e.g. 'transforms[0].header.'
    """
    if not _is_ros_msg(obj) and not hasattr(obj, "__slots__"):
        return

    for field_name in _iter_field_names(obj):
        try:
            val = getattr(obj, field_name)
        except Exception:
            continue

        if val is None:
            continue

        visit_field(obj, field_name, val, path)

        # Re-fetch in case visit_field changed it.
        try:
            val = getattr(obj, field_name)
        except Exception:
            continue

        if val is None:
            continue

        if _is_sequence(val):
            for i, item in enumerate(val):
                walk_ros_message(item, visit_field=visit_field, path=f"{path}{field_name}[{i}].")
        else:
            walk_ros_message(val, visit_field=visit_field, path=f"{path}{field_name}.")


# ======================================================================
#  Transformations
# ======================================================================


def remap_frames_in_msg(
    msg: Any,
    *,
    direction: str,
    prefix_token: str,
    exclude_frames: set[str],
    collect_samples: int = 0,
) -> Tuple[Any, int, List[Tuple[str, str, str]]]:
    """
    Prefixes / un-prefixes all occurrences of:
    - frame_id
    - child_frame_id
    anywhere inside a ROS2 Python message (including nested messages and arrays).

    Mutates msg in-place.
    Returns: (msg, number_of_changes, samples[path, old, new])
    """
    if direction not in ("add", "remove"):
        raise ValueError(f"direction must be 'add' or 'remove', got '{direction}'")

    samples: List[Tuple[str, str, str]] = []

    def record(path: str, old: str, new: str) -> None:
        if collect_samples > 0 and len(samples) < collect_samples:
            samples.append((path, old, new))

    def is_excluded(frame_id: str) -> bool:
        if not frame_id:
            return True
        f = frame_id.lstrip("/")
        return frame_id in exclude_frames or f in exclude_frames

    def add_prefix(frame_id: str) -> str:
        if not frame_id or not prefix_token or is_excluded(frame_id):
            return frame_id
        if frame_id.startswith(prefix_token):
            return frame_id
        return f"{prefix_token}{frame_id}"

    def remove_prefix(frame_id: str) -> str:
        if not frame_id or not prefix_token or is_excluded(frame_id):
            return frame_id
        if frame_id.startswith(prefix_token):
            return frame_id[len(prefix_token) :]
        return frame_id

    fn = add_prefix if direction == "add" else remove_prefix
    changes = 0

    def visit_field(obj: Any, field_name: str, val: Any, path: str) -> None:
        nonlocal changes
        if field_name in ("frame_id", "child_frame_id") and isinstance(val, str):
            old = val
            new = fn(old)
            if new != old:
                setattr(obj, field_name, new)
                changes += 1
                record(f"{path}{field_name}", old, new)

    walk_ros_message(msg, visit_field=visit_field, path="")
    return msg, changes, samples


def update_timestamps_in_msg(
    msg: Any,
    *,
    new_stamp: Any,
    collect_samples: int = 0,
) -> Tuple[Any, int, List[Tuple[str, str, str]]]:
    """
    Rebase all fields named 'stamp' (e.g. builtin_interfaces/msg/Time) to `new_stamp`,
    anywhere inside a ROS2 Python message (including nested messages and arrays).

    Mutates msg in-place.
    Returns: (msg, number_of_changes, samples[path, old, new])
    """
    samples: List[Tuple[str, str, str]] = []

    def record(path: str, old: Any, new: Any) -> None:
        if collect_samples > 0 and len(samples) < collect_samples:
            samples.append((path, _format_time(old), _format_time(new)))

    changes = 0

    def visit_field(obj: Any, field_name: str, val: Any, path: str) -> None:
        nonlocal changes
        if field_name != "stamp":
            return

        # scalar Time-like
        if _is_time_msg(val):
            old = val
            if old != new_stamp:
                setattr(obj, field_name, new_stamp)
                changes += 1
                record(f"{path}{field_name}", old, new_stamp)
            return

        # list/tuple of Time-like
        if _is_sequence(val) and val and all(_is_time_msg(x) for x in val):
            if isinstance(val, list):
                for i in range(len(val)):
                    if val[i] != new_stamp:
                        record(f"{path}{field_name}[{i}]", val[i], new_stamp)
                        val[i] = new_stamp
                        changes += 1
                return

            if isinstance(val, tuple):
                # tuples are immutable; replace the whole field
                new_tuple = tuple(new_stamp for _ in val)
                if new_tuple != val:
                    setattr(obj, field_name, new_tuple)
                    changes += 1
                    record(f"{path}{field_name}", val, new_tuple)

    walk_ros_message(msg, visit_field=visit_field, path="")
    return msg, changes, samples





