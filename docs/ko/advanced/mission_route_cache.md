# Mission Route Cache

Normal mission execution keeps a small sliding window of mission items in RAM.
Route-aware features may instead need random, non-blocking access to the complete route.

The mission route cache provides this data without reading Dataman or the SD card during planning:

```text
MissionRouteCache
|-- full mission route (optional) [0, ..., CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE - 1]
|-- safe points                  [all uploaded safe points]
`-- published mission-land item  [one item]
```

The cache is infrastructure for Navigator features.
It does not change normal mission execution by itself.

## 구매처

The full route is loaded into RAM asynchronously.
It becomes available only after every mission item has loaded successfully. A partial route is never exposed.

When the cached mission identity changes, the old route becomes unavailable immediately and the replacement is loaded as a new generation.
Failed reads keep the successfully loaded prefix and retry with a capped backoff.

:::info
If the full cache is disabled or the mission exceeds its capacity, a consuming feature must use its normal fallback behavior.
The independent safe-point and mission-land caches remain available.
:::

## 설정

`CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE` sets the maximum number of items in the full route cache.
It defaults to `500` on POSIX platforms such as SITL, and `0` on embedded targets unless a board overrides it.

A value of `0` removes the full-mission buffer and its Dataman client from the build.
Boards that enable full-route planning must choose a capacity that fits their RAM budget.

The buffer is allocated once when Navigator starts and uses one `mission_item_s` entry per configured item.
An entry is currently 56 bytes, so 500 items use 28 kB.

:::details
Developer details: Loading workflow

The full mission is exposed as one complete generation:

```text
New mission accepted
        ↓
Read one item asynchronously
        ↓
Dataman response wakes Navigator
        ↓
Validate the response and queue the next item
        ↓
Repeat until every item is loaded
        ↓
Mark the complete generation ready
```

Only one full-mission read is outstanding at a time.
While it is pending, Navigator polls that client's response subscription, so each completion can queue the next read immediately.
There is no separate cache thread.

The response topic is shared by all Dataman clients, so poll readiness alone does not mean this request completed.
The response is still copied and matched to the request, and the subscription is polled only while this cache has a pending read.

Cache-only wakeups advance the load without running all Navigator state machines at the Dataman response rate.
During a burst of Dataman-only wakeups, normal Navigator work stays on its existing cadence; a cache wake also runs it once the update period is due.

The cache identifies a mission source by its mission ID, item count, and Dataman bank.
If the source changes while a read is in flight, that response is consumed but not accepted into the new generation.

:::

:::details
Developer details: Borrowing a mission view

`getMissionView()` returns a zero-copy view of the cache-owned array.
The view contains a pointer, item count, mission identity, and generation number; it does not own or copy the items.

A consumer should acquire a complete view, compute synchronously, and validate the same view before accepting the result:

```text
Acquire view → compute route → generation still valid?
                                      |
                         yes ─────────┴──────── no
                          |                       |
                    accept result          discard result
                                           acquire new view
                                           recompute
```

The caller does not update the pointer inside a stale view.
It discards the view and anything derived from it, then calls `getMissionView()` again and recomputes.

After an item in a ready generation is patched, a fresh view is available immediately.
While a replacement mission is loading, acquiring a new view fails until the complete generation is ready.

The refreshed view may contain the same pointer address because the backing allocation is reused.
The generation, not pointer equality, determines whether its contents are current.

Use a borrowed view only during serialized Navigator work, and check it with `missionViewStillValid()` before accepting the result.
The generation check is not a lock; another task would need explicit synchronization or its own snapshot.

:::

:::details
Developer details: Keeping mission writes coherent

Already-loaded mission items are not automatically read from storage again.
After a successful active-mission Dataman write, the writer must call `syncMissionItem()` from Navigator's task.
This call synchronizes the caches; it does not perform the Dataman write.

For a ready full cache, synchronization patches that item in the array and advances the generation:

```text
Successful Dataman write
        ↓
Patch the cached item
        ↓
Advance the generation
        ↓
Old views and route results are stale
```

The complete route stays ready.
Reloading the whole mission for a one-item change would make it unavailable and add one read per mission item without improving consistency.

Writes during loading depend on how far the load has progressed:

- An item in the loaded prefix is patched in place.
- A response for the item already being read is ignored, and that item is then read again.
- An unread item is picked up later by the normal load.

Mission execution follows this process after successful `DO_JUMP` counter increments and resets.
The next route computation therefore sees the current number of remaining loops.

The mission-land cache is synchronized independently when the written index is the published land item.

A mission upload or replacement is different from an in-place runtime write.
Its new mission identity causes a complete generation to load.

:::

:::details
Developer details: Measuring load time

Successful, non-empty loads are reported by the `navigator: full mission cache load` performance counter.
It measures wall time until the complete route is ready, including retry delays.

To measure it on hardware:

```sh
perf reset
# Upload or change the mission.
perf
```

Replaced, invalidated, empty, or incomplete loads do not add a sample.

:::
