# CAN → USB throughput investigation (rhs/fpcan)

Investigation into ~4% CAN frame loss on `rhs/fpcan` (STM32F405) under an RTR flood
at 1 Mbit/s from a PEAK CAN adapter, and the counters added to measure exactly where
frames are being lost before changing any pipeline behavior.

Note: this board is STM32F405 (`Kconfig.fpcan: select SOC_STM32F405XX`,
`fpcan.dts:8`), not STM32F103 — the pipeline code discussed below (`gs_usb.c`,
`can_stm32_bxcan.c`, `udc_stm32.c`) is shared across STM32 bxCAN parts, so the
findings apply, but board-specific theories (e.g. an F103 CAN/USB shared-IRQ
vector) do not: on F405, CAN1 uses IRQ 19-22 and USB OTG FS uses IRQ 67
(`zephyr/dts/arm/st/f4/stm32f405.dtsi:224-227`,
`zephyr/dts/arm/st/f4/stm32f4.dtsi:362-373`) — independent vectors, ruled out.

## Pipeline and drop/stall points

```
CAN wire → bxCAN FIFO0 IRQ (can_stm32_rx_isr, ISR context)
  → gs_usb_can_rx_callback (still ISR context, gs_usb.c:1115)
      → net_buf_alloc_fixed(pool, K_NO_WAIT)      ← DROP POINT A
      → k_fifo_put(rx_fifo)
  → gs_usb_rx_thread (prio 0, preemptible, gs_usb.c:1191)
      → k_fifo_get(rx_fifo, K_FOREVER)
      → usbd_ep_enqueue()
      → k_sem_take(in_sem, K_FOREVER)              ← SERIALIZATION POINT (no pipelining)
  → udc_stm32 USB ISR → k_msgq_put(msgq_data, depth 8, K_NO_WAIT)  ← DROP POINT B (can stall the point above forever — no timeout)
  → udc_stm32_thread_handler (coop prio 8) → gs_usb_request() → k_sem_give(in_sem)
```

1. **No pipelining on bulk-IN.** `gs_usb_rx_thread()`
   (`subsys/usb/device_next/class/gs_usb.c:1191-1253`) submits one CAN frame as one
   `usbd_ep_enqueue()` call, then blocks on `k_sem_take(&data->in_sem, K_FOREVER)`
   (`gs_usb.c:1238`) until that transfer's completion fires. Only one host frame is
   ever in flight on the endpoint — every CAN frame costs a full USB round trip.
   Most likely dominant cap on sustained frame rate.

2. **Shared, small `net_buf` pool.** `CONFIG_USBD_GS_USB_POOL_SIZE` defaults to 20
   (21 with compatibility mode) — `subsys/usb/device_next/class/Kconfig.gs_usb:29-36`.
   One pool backs RX frames, TX-echo frames, and error/state frames for all channels
   (fpcan wires up both CAN1 and CAN2, `fpcan.dts:92-102`, competing for the same
   pool). `gs_usb_can_rx_callback()` allocates with `K_NO_WAIT` (`gs_usb.c:958`); on
   failure the frame is simply dropped.

3. **Only bxCAN FIFO0 is used.** FIFO1 (`zephyr/drivers/can/can_stm32_bxcan.c:970`)
   sits idle with its own 3-mailbox hardware buffer unused, halving the real hardware
   burst absorption bxCAN offers before the ISR must run.

4. **No frame batching.** A classic-CAN host frame is 20-24 bytes; a full-speed USB
   packet is 64 bytes (room for 2-3 frames), but each frame gets its own `net_buf`
   and its own `usbd_ep_enqueue()` call (`gs_usb.c:1230`).

5. **UDC message queue can silently drop completion events.**
   `CONFIG_UDC_STM32_MAX_QMESSAGES` defaults to 8
   (`zephyr/drivers/usb/udc/Kconfig.stm32:33-38`). If a bulk-IN completion event is
   dropped this way, point 1's `k_sem_take(..., K_FOREVER)` never returns and the
   whole RX→USB path can stall indefinitely.

These are candidate fixes for a follow-up pass; **this pass only adds
instrumentation**, so each candidate can be prioritized against a measured
baseline rather than guessed at.

## Instrumentation added

| Drop point | Location | What was added |
|---|---|---|
| A — gs_usb `net_buf` pool exhaustion | `subsys/usb/device_next/class/gs_usb.c` | `atomic_t rx_drops` added to `struct gs_usb_channel_data` (line ~58); incremented and logged at both allocation-failure sites: `gs_usb_can_rx_callback()` (~line 1141-1146) and `gs_usb_can_state_change_callback()` (~line 1033-1038). Reset alongside the existing `k_sem_reset(&channel->rx_overflows)` in `gs_usb_reset_channel()`. Existing `GS_USB_CAN_FLAG_OVERFLOW` host-visible behavior is unchanged — this only adds a running total to the log line. |
| B — UDC completion-event queue overrun | `drivers/usb/udc/udc_stm32.c` (**west-managed `zephyr` module — see note below**) | `atomic_t msgq_overruns` added to `struct udc_stm32_data`; incremented and logged at all three `k_msgq_put(..., K_NO_WAIT)` failure sites (`HAL_PCD_DataOutStageCallback`, `HAL_PCD_DataInStageCallback`, `HAL_PCD_SetupStageCallback`). |
| Hardware FIFO0 overrun (already existed) | `drivers/can/can_stm32_bxcan.c:150-153` (zephyr module) | Already counted via `CAN_STATS_RX_OVERRUN_INC(dev)` and already logs `"RX FIFO Overflow"` per event at `LOG_ERR` level — no code change needed. `CONFIG_CAN_STATS=y` was added via `app/boards/fpcan_stm32f405xx.conf` so the running total is tracked (queryable later via `can_stats_get_rx_overruns()` if a shell/API is added). |

**Important — the `zephyr/` directory is a separate, unmodified upstream checkout**
(`west.yml` pulls `zephyr` from `zephyrproject-rtos/zephyr` at `revision: main`, not a
fork). The edits to `drivers/usb/udc/udc_stm32.c` and the note about
`drivers/can/can_stm32_bxcan.c` are **not tracked by the `cannectivity` git repo** and
will be **wiped by `west update`**. Save them as a patch (e.g.
`git -C ../zephyr diff > udc_stm32_instrumentation.patch`) before running `west
update`, or keep this local working copy until the measurement pass is done.

All new log lines use `LOG_ERR`, consistent with the existing `"RX FIFO Overflow"`
and `"failed to allocate RX host frame"` lines they sit next to, so they show up with
the same logging config already used for debugging (see `app/prj_debug.conf`, which
sets `CONFIG_LOG=y` and per-module debug levels — no new logging config needed for
`LOG_ERR` visibility, though enabling `prj_debug.conf` is recommended when running the
flood test).

## Build note (unrelated pre-existing issues found while verifying)

While confirming these changes compile, two **pre-existing, unrelated** issues in
this workspace were found — neither caused by this instrumentation:

- ~~`app/src/usb.c:303` and `subsys/usb/device_next/class/gs_usb.c:1760` fail to
  build...~~ **Fixed.** `USBD_DESC_BOS_VREQ_DEFINE` / `gs_usb_api.control_to_host`
  expected a `to_host` callback returning `int` with a pre-allocated `net_buf`
  parameter, but this tree still had the older signature (self-allocating,
  returning `struct net_buf *`) — a Zephyr USBD vendor-request API change not yet
  adapted to here. `cannectivity_usb_msosv2_handler` (`app/src/usb.c`) and
  `gs_usb_control_to_host` plus its six sub-handlers (`gs_usb.c`) were converted
  to the new signature, modeled on `zephyr/samples/subsys/usb/webusb/src/msosv2.h`.
  Verified via a full `lpcxpresso55s16` build.
- `west build -b rhs/fpcan` board resolution is blocked by a duplicate board
  definition — an **untracked** `zephyr/boards/rhs/fpcan/` directory (byte-identical
  copy of `boards/rhs/fpcan/`, sitting inside the west-managed `zephyr` module
  checkout) collides with the real board dir under `cannectivity/boards/rhs/`.
  Left as-is per your call — needs to be removed from the `zephyr` checkout before
  `rhs/fpcan` will resolve.

The board-resolution issue was still present at the time of the flood-test run
below, so that build/flash must have gone through a workaround (e.g. a temporarily
adjusted `BOARD_ROOT`, or the duplicate having been cleared locally and not yet
regenerated) — not reproduced or investigated further here.

## Running the measurement

Once the two build issues above are resolved:

1. Build with `app/prj_debug.conf` layered in so `LOG_ERR` output is visible.
2. Run the PEAK RTR-flood test at 1 Mbit/s as before, capturing firmware log
   output alongside the host-side (candump / gs_usb driver) observed frame count.
3. Record, from the same run:
   - gs_usb pool-exhaustion drops (`rx_drops` total, from the log line in
     `gs_usb.c`).
   - UDC msgq overruns (`msgq_overruns` total, from the log line in
     `udc_stm32.c`).
   - Hardware FIFO0 overruns ("RX FIFO Overflow" occurrences in the log).
   - Host-observed frame loss count/percentage for the same run.
4. Compare: if the three counters' sum accounts for the host-observed loss, the
   dominant drop point tells you which of findings #1-#5 above to fix first (e.g.
   pool exhaustion dominant → prioritize #1/#2; msgq overruns significant →
   prioritize #5). If they don't add up, there's an undiscovered drop point to
   chase.

## Results (2026-08-12, `rhs/fpcan`, 1 Mbit/s RTR flood, 30 s run)

Host side (PEAK adapter / test harness):

| Metric | Value |
|---|---|
| Frames sent | 343,477 (11,449.2 fps) |
| Frames received | 323,842 (10,794.7 fps) |
| Frames missed | 19,635 |
| Loss | 5.72% |

Firmware side (RTT log, `gs_usb.c` `rx_drops` counter, channel 0):

- Last captured value at end of run: **`total drops: 18398`**.
- The RTT log itself was dropping messages under load (`--- 20 messages dropped
  ---`, `--- 15 messages dropped ---` interleaved between `rx_drops` log lines),
  so the true end-of-run count is **≥ 18398** — the logging channel cannot
  fully keep up with the drop rate either, though each surviving log line still
  carries the correct running total via the atomic counter, so no count is lost,
  only intermediate log lines.
- No UDC msgq-overrun or "RX FIFO Overflow" log lines were reported from this
  run.

**Conclusion**: gs_usb `net_buf` pool exhaustion (drop point A, finding #2)
accounts for **≥ 93.7%** (18398 / 19635) of observed frame loss on its own, with
the gap plausibly closed by further drops during the RTT log's own dropped
message windows rather than by a second drop point. This matches the plan's
predicted profile for "pool exhaustion dominant": findings **#1 (no bulk-IN
pipelining — one frame per USB round trip, serialized on
`k_sem_take(..., K_FOREVER)`)** and **#2 (small, shared, `K_NO_WAIT` pool)** are
confirmed as the priority fixes. Findings #3 (FIFO1 unused) and #5 (UDC msgq
overrun) are not implicated by this run and can be deprioritized unless a
future run shows otherwise.

## Fix pass 1: pipeline bulk-IN + grow the pool (findings #1 + #2)

Implementing both together, since fixing #1 alone (allowing several frames in
flight) would just move the exhaustion point to the pool if the pool stays at
its current size:

- **`gs_usb_data.in_sem`** (`gs_usb.c`) changes from a binary semaphore
  (0..1, "one bulk-IN transfer in flight") to a counting semaphore sized by a
  new `CONFIG_USBD_GS_USB_MAX_IN_FLIGHT` Kconfig option. `gs_usb_rx_thread()`
  now takes a permit *before* `usbd_ep_enqueue()` instead of *after* — so it
  only blocks once `MAX_IN_FLIGHT` transfers are already outstanding, instead
  of after every single one. The completion callback (`gs_usb_request()`)
  already gives the permit back; that part is unchanged.
- **`CONFIG_USBD_GS_USB_POOL_SIZE`** is raised for `rhs/fpcan` specifically
  (via `app/boards/fpcan_stm32f405xx.conf`) so the deeper in-flight pipeline
  and the two-channel (CAN1+CAN2) sharing of one pool both have enough
  buffers to avoid immediately re-creating the exhaustion this pass just
  fixed.

Findings #3 (FIFO1) and #5 (UDC msgq depth) stay out of scope — the 2026-08-12
run gives no evidence they're needed.

## Results, fix pass 1 (2026-08-12, `rhs/fpcan`, pool=64, in-flight=8)

| Metric | Baseline | Pass 1 |
|---|---|---|
| Frames sent | 343,477 (11,449.2 fps) | 338,778 (11,292.6 fps) |
| Frames received | 323,842 (10,794.7 fps) | 328,464 (10,948.8 fps) |
| Frames missed | 19,635 | 10,314 |
| Loss | 5.72% | 3.04% |
| gs_usb `rx_drops` at end of run | ≥ 18,398 | 10,150 (log line: `total drops: 10150`) |

Pool exhaustion is still **~98.4%** of the loss (10,150 / 10,314), i.e. it
remains the single dominant, essentially sole, drop point — pipelining alone
roughly halved the loss, as expected, but didn't touch the underlying
resource constraint.

## Fix pass 2 (2026-08-12): push pool size and in-flight depth further

Build memory report for `rhs/fpcan` with pass 1's settings showed plenty of
headroom (FLASH 11.30%, RAM 24.78% of 128 KB) — the pool is a small `net_buf`
allocation, not a meaningful fraction of available SRAM even at several times
its pass-1 size. Since pool exhaustion is still ~98% of the remaining loss,
the next cheap experiment is simply pushing both knobs further to see how
much more of the loss buffering/pipelining depth alone can absorb before
hitting a different bottleneck (e.g. sustained USB FS bulk-IN throughput,
which pipelining depth cannot fix beyond what the wire actually supports):

- `CONFIG_USBD_GS_USB_POOL_SIZE`: 64 → **256**
- `CONFIG_USBD_GS_USB_MAX_IN_FLIGHT`: 8 → **32**

Sanity-checked by building with these values layered onto `nucleo_h723zg`
(large-RAM STM32H7 board, since `rhs/fpcan` itself is still blocked by the
board-name collision) — links clean, RAM usage 63,516 B / 320 KB (19.38%).
Not yet build-verified for `rhs/fpcan`'s actual 128 KB SRAM budget, but the
pass-1 → pass-2 pool delta (64 → 256, +192 buffers) cost roughly 40 KB on a
smaller board in an earlier check, which extrapolated onto fpcan's baseline
24.78% (32,476 B) usage would land around 55-60% RAM used — comfortable, but
worth confirming once the board resolves.

If loss drops close to zero, pipelining/pooling was the whole story. If loss
plateaus well above zero despite drops no longer being pool-exhaustion (i.e.
`rx_drops` stops explaining most of the remaining loss), that's the signal to
look at findings #3 (FIFO1 unused) or the sustained bulk-IN throughput ceiling
itself rather than continuing to grow these two knobs.

## Results, fix pass 2 (2026-08-13, `rhs/fpcan`, pool=256, in-flight=32)

Also fixed alongside this measurement: `CONFIG_CAN_STATS=y` had been silently
dropped by Kconfig this whole time because it `depends on STATS`
(`zephyr/drivers/can/Kconfig:81-85`) and `CONFIG_STATS` was never enabled —
`can show <device>` printed no `statistics:` block at all. Added
`CONFIG_STATS=y` to `app/boards/fpcan_stm32f405xx.conf` so the hardware RX
overrun counter is actually visible via `can show`.

| Metric | Baseline | Pass 1 | Pass 2 |
|---|---|---|---|
| Frames sent | 343,477 (11,449.2 fps) | 338,778 (11,292.6 fps) | 348,802 (11,626.7 fps) |
| Frames received | 323,842 (10,794.7 fps) | 328,464 (10,948.8 fps) | 341,659 (11,388.6 fps) |
| Frames missed | 19,635 | 10,314 | 7,143 |
| Loss | 5.72% | 3.04% | 2.05% |
| gs_usb `rx_drops` at end of run | ≥ 18,398 | 10,150 | ~6,808 (mid-run; consistent with final count) |
| bxCAN hardware RX overruns (`can show`) | not measured | not measured | **0** |

Pool size and in-flight depth going 4x each (pass 1 → pass 2) only bought a
~31% reduction in misses (10,314 → 7,143), a shrinking return relative to the
pass-1 gain, while `rx_drops` still explains essentially all of the loss. With
hardware RX overruns confirmed at 0, the drop is entirely on the gs_usb/USB
side — findings #3 (FIFO1 unused) is ruled out, since the hardware isn't the
constraint.

## Fix pass 3 (2026-08-13): fix the UDC completion-event queue mismatch

Finding #4 (batching multiple CAN frames into one USB bulk-IN transfer) was
considered next and **rejected**: the Linux `gs_usb` kernel driver (and
`python-can`) expects exactly one host frame per bulk-IN transfer/URB. Packing
frames would break interoperability with real host tooling rather than fix a
bottleneck — not worth it even if it were faster.

Instead, re-examining finding #5 in light of pass 2's diminishing returns
turned up a mismatch introduced by pass 1/2 themselves:
`CONFIG_USBD_GS_USB_MAX_IN_FLIGHT=32` allows up to 32 bulk-IN transfers
outstanding at once, but every one of those completions has to pass through
the STM32 UDC driver's own ISR event queue —
`CONFIG_UDC_STM32_MAX_QMESSAGES` (`zephyr/drivers/usb/udc/Kconfig.stm32:33-38`),
which still defaults to **8** and was never raised alongside the in-flight
depth above it. That queue put is `k_msgq_put(..., K_NO_WAIT)` — silent on
overrun — and losing a completion event there means the corresponding
`gs_usb` `in_sem` permit is never given back: a leaked permit that
permanently shrinks the effective in-flight window for the rest of the run.
This is a better explanation for pass 2's shrinking returns than pool size
itself: buffers and in-flight depth were scaled up 4x, but the fixed 8-deep
queue immediately downstream of that pipeline was not, so it's a strong
candidate for the actual ceiling.

- `CONFIG_UDC_STM32_MAX_QMESSAGES`: 8 → **64** (the Kconfig maximum, so it's
  never smaller than `MAX_IN_FLIGHT` above). Each queued message is 4 bytes
  (`struct udc_stm32_msg`: `type`, `ep`, `rx_count`), so this costs 224 bytes —
  negligible next to the SRAM headroom already established.

Sanity-checked by building with all of `app/boards/fpcan_stm32f405xx.conf`
layered onto `nucleo_h723zg` (`rhs/fpcan` itself still blocked by the
board-name collision) — links clean, RAM usage 63,836 B / 320 KB (19.48%).

If this closes most of the remaining ~2% loss, the UDC event queue was the
real ceiling, not raw buffer/pipeline capacity. If loss barely moves, the
bottleneck has shifted to sustained USB FS bulk-IN throughput itself (the wire
rate), which no amount of buffering, in-flight depth, or queue depth can fix —
at that point the investigation should stop tuning knobs and instead measure
the actual bulk-IN transaction rate against the theoretical full-speed USB
ceiling (~19,000 64-byte transactions/sec) to confirm.

**Pass 3 result: loss barely moved** (still ~2%, same order as pass 2). Two
follow-ups from that:

1. **Config bug found while re-reading the file (2026-08-13)**:
   `CONFIG_USBD_GS_USB_MAX_IN_FLIGHT` was found reading `3`, not the intended
   `32`, in `app/boards/fpcan_stm32f405xx.conf`. Not caused by any edit made
   here (no diff touched that line) — the file is untracked
   (`git status`), so there's no history to attribute it to. Fixed back to
   `32`. Worth being aware this may have silently capped in-flight depth to 3
   for some unknown portion of the pass-2/pass-3 runs, which would itself
   largely explain pass 3's flat result (a fixed 8→64-deep completion queue
   downstream of an already-3-deep pipeline has nothing to help with).
2. **Host-side ceiling suspected.** ~11,388 fps received in pass 2 is only
   ~88 µs/frame — far more than the actual USB bus time a 20-byte host frame
   needs (single-digit µs). That gap is consistent with the *host's* USB
   stack/driver pacing IN-token issuance (e.g. the receiving `gs_usb`/
   `python-can` driver's own fixed RX URB pool depth), not anything left to
   tune device-side. A host-side USB capture (macOS: Wireshark's built-in
   per-controller capture interfaces, e.g. `XHC20`) during the flood, looking
   at bulk-IN transaction spacing and how many are outstanding at once, would
   confirm or rule this out. Not something this firmware can fix if
   confirmed.

## Instrumentation correctness fix (2026-08-13): counters vs. disabled logging

`rx_drops` (`gs_usb.c`) and `msgq_overruns` (`udc_stm32.c`) were incremented
*inside* the `LOG_ERR(...)` call's arguments, e.g.:

```c
LOG_ERR("failed to allocate RX host frame for channel %u (total drops: %u)",
	channel->ch, (unsigned int)atomic_inc(&channel->rx_drops) + 1U);
```

Zephyr's `LOG_ERR` macro (`Z_LOG_LEVEL_ALL_CHECK_BREAK`,
`zephyr/include/zephyr/logging/log_core.h:215-218`) `break`s out before
evaluating `__VA_ARGS__` whenever the call is filtered/compiled out — so with
logging disabled, the `atomic_inc()` embedded as a log argument would never
run, silently breaking both counters instead of just silencing their output.
Fixed at all five call sites (two in `gs_usb.c`, three in `udc_stm32.c`) by
moving the increment to its own statement before the (now argument-only)
`LOG_ERR` call, so the counters keep working regardless of logging config.

`CONFIG_LOG` is then set to `n` in `app/boards/fpcan_stm32f405xx.conf` to
measure without the RTT/logging overhead the per-drop `LOG_ERR` calls add
under sustained loss (hundreds of RTT writes/sec at pass-2/3 drop rates,
competing with the pipeline being measured). `CONFIG_SHELL`/`CAN_SHELL`/RTT
stay enabled (`SHELL` has no dependency on `LOG`), so `can show` still works
for the hardware-overrun check between runs — there's just no way to read
`rx_drops`/`msgq_overruns` without re-enabling `CONFIG_LOG=y` temporarily, in
the absence of a dedicated shell command for them.
