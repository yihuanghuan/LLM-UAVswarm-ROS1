"""Reject stale hover-stable samples across command generations."""
from __future__ import annotations


class CompletionGenerationTracker:
    def __init__(self):
        self._seen_unstable = {}
        self._stable = {}

    def arm(self, uav_ids):
        for uid in uav_ids:
            self._seen_unstable[int(uid)] = False
            self._stable[int(uid)] = False

    def update(self, uav_id: int, is_stable: bool):
        uid = int(uav_id)
        if uid not in self._seen_unstable:
            return
        if not is_stable:
            self._seen_unstable[uid] = True
            self._stable[uid] = False
        elif self._seen_unstable[uid]:
            self._stable[uid] = True

    def is_stable(self, uav_id: int) -> bool:
        return self._stable.get(int(uav_id), False)

    def all_stable(self, uav_ids) -> bool:
        return all(self.is_stable(uid) for uid in uav_ids)
