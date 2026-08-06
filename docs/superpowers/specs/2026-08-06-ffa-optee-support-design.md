# FF-A (Arm Firmware Framework for A-profile) support in Zephyr

- **Date:** 2026-08-06
- **Status:** Approved design, pending implementation
- **Target:** Zephyr on Cortex-A (AArch64), Normal-World endpoint talking to OP-TEE
- **Reference:** Linux kernel `drivers/firmware/arm_ffa/` and `drivers/tee/optee/ffa_abi.c`

## Goal

Add Arm **FF-A** (Firmware Framework for A-profile) support to Zephyr so that a
Zephyr instance running as the Normal-World endpoint on Cortex-A can communicate
with **OP-TEE** over the FF-A ABI — as an alternative to the existing raw SMC
transport. Today Zephyr reaches M-profile secure world via FF-M/TF-M and reaches
A-profile OP-TEE only over the legacy `OPTEE_SMC_*` ABI. This work adds the
A-profile FF-A path.

The FF-A layer is built as a **reusable subsystem**, not glue folded into the
OP-TEE driver, so other secure-partition consumers can use it later.

### In scope

- A reusable FF-A core subsystem (version negotiation, partition discovery,
  direct messaging, memory sharing, notifications).
- FF-A framework version **1.2** negotiation (graceful downgrade to 1.1/1.0).
- An OP-TEE **FF-A transport** layered on the core, added alongside the existing
  SMC transport (which must keep working).
- Validation on **`qemu_cortex_a53` + OP-TEE**.

### Out of scope

- Zephyr acting as a Secure Partition / SPMC (Zephyr is the NS endpoint only).
- Indirect messaging (`FFA_MSG_SEND2` / RX-TX indirect) — OP-TEE does not use it.
- A full Linux-style `ffa_device`/`ffa_driver` bus with partition-UUID driver
  matching. We expose a direct API instead (see "Rejected alternatives").
- Non-QEMU targets (FVP/Hafnium) — the design must not preclude them, but they
  are not validated here.

## Working assumptions

- **Role:** Zephyr is the Normal-World (NS) FF-A endpoint. OP-TEE is the secure
  partition it messages. The conduit is `SMC` (or `HVC` when virtualized),
  selected from devicetree, exactly as the existing OP-TEE driver already does.
- **OP-TEE-over-FF-A message protocol:** OP-TEE in this tree advertises its
  OP-TEE FF-A protocol as v1.0 while speaking FF-A framework 1.0/1.1/1.2. The
  FF-A core negotiates the framework version; the OP-TEE transport negotiates
  the OP-TEE protocol version via `OPTEE_FFA_GET_API_VERSION`.
- **Conduit register width:** FF-A 1.2 calls (`MSG_SEND_DIRECT_REQ2`,
  `PARTITION_INFO_GET_REGS`, `NOTIFICATION_INFO_GET`) use SMCCC 1.2 extended
  registers (x0–x17). Zephyr's current SMCCC layer carries only x0–x7. This is a
  hard prerequisite (see SP-1).

## Key constraint discovered in the code

`zephyr/include/zephyr/arch/arm64/arm-smccc.h` defines `struct arm_smccc_res`
with only `a0..a7`, and `arm_smccc_smc/hvc` (`arch/arm64/core/smccc-call.S`)
pass only 8 registers. Full FF-A 1.2 needs **SMCCC 1.2 extended registers
(x0–x17)**. Nothing in the FF-A core works without this, so it is the first
sub-project.

## Architecture

Bottom-up layering:

```
+-----------------------------------------------------------+
|  Consumers: TEE API (existing), future FF-A partitions    |
+-----------------------------------------------------------+
|  OP-TEE driver: transport-ops abstraction                 |
|    - optee_smc  transport (existing SMC path, unchanged)  |
|    - optee_ffa  transport (new, uses FF-A core)           |
+-----------------------------------------------------------+
|  FF-A core subsystem  (subsys/firmware/ffa)               |
|    setup | discovery+messaging | memory sharing | notif   |
|    public API: include/zephyr/firmware/ffa.h              |
+-----------------------------------------------------------+
|  SMCCC 1.2 call layer (arch/arm64)                        |
|    arm_smccc_1_2_regs (x0..x17) + arm_smccc_1_2_smc/hvc   |
+-----------------------------------------------------------+
|  EL3 / SPMD / SPMC  ->  OP-TEE (secure partition)         |
+-----------------------------------------------------------+
```

### Component 1 — SMCCC 1.2 call layer (`arch/arm64`)

- New `struct arm_smccc_1_2_regs { unsigned long a0..a17; }` in
  `arm-smccc.h` (kept separate from the existing 8-register `arm_smccc_res`; the
  existing struct and `arm_smccc_smc/hvc` are unchanged so PSCI and the SMC
  OP-TEE path are untouched).
- New assembly `arm_smccc_1_2_smc(const struct arm_smccc_1_2_regs *args,
  struct arm_smccc_1_2_regs *res)` and `_hvc` in `smccc-call.S`, saving/
  restoring x0–x17 per SMCCC 1.2.
- Gated by a new Kconfig `CONFIG_ARM_SMCCC_1_2` (arm64 only).

**What it does / interface / deps:** marshals up to 18 registers through an
SMC/HVC. Depends only on the AArch64 exception model. Consumed by the FF-A core.

### Component 2 — FF-A core subsystem (`subsys/firmware/ffa/`)

Public header `include/zephyr/firmware/ffa.h`. Internal ABI constants in
`subsys/firmware/ffa/ffa_internal.h` (function IDs, error codes, descriptors),
ported from `linux/include/linux/arm_ffa.h`. Split into focused files:

- `ffa_core.c` — init, conduit detect, `FFA_VERSION` negotiate (offer 1.2,
  accept the SPMC's reply, downgrade), `FFA_ID_GET`, `FFA_FEATURES` probe,
  RX/TX buffer allocation + `FFA_RXTX_MAP`, central `FFA_ERROR` → `-errno`
  decode. Owns the singleton driver state and a mutex serializing RX buffer use.
- `ffa_msg.c` — `FFA_PARTITION_INFO_GET[_REGS]` (with RX-buffer and register
  variants; strips the FF-A 1.0 8-byte partition record when negotiated version
  is 1.0), and `MSG_SEND_DIRECT_REQ`/`REQ2` + RESP round-trips.
- `ffa_mem.c` — composite memory-region descriptor construction, `MEM_SHARE`
  (native FN64), `MEM_FRAG_TX` fragmentation for large descriptors, and
  `MEM_RECLAIM`.
- `ffa_notif.c` — `NOTIFICATION_BITMAP_CREATE/DESTROY`, `BIND`/`UNBIND`,
  `SET`/`GET`, `INFO_GET`, and schedule-receiver handling (interrupt if a DT
  interrupt is present, else a poll fallback), dispatching to registered
  callbacks.

**Public API (mirrors Linux `ffa_ops`, flattened to functions):**

```c
/* setup / discovery */
int  ffa_version(uint32_t *out);              /* negotiated version   */
int  ffa_id_get(uint16_t *vm_id);
int  ffa_partition_info_get(const struct ffa_uuid *uuid,
                            struct ffa_partition_info *out, size_t *count);
/* messaging */
int  ffa_msg_send_direct_req(uint16_t dst, struct ffa_send_direct_data *d);
int  ffa_msg_send_direct_req2(uint16_t dst, const struct ffa_uuid *uuid,
                              struct ffa_send_direct_data2 *d);
/* memory */
int  ffa_mem_share(struct ffa_mem_ops_args *args);   /* fills args->g_handle */
int  ffa_mem_reclaim(uint64_t g_handle, uint32_t flags);
/* notifications */
int  ffa_notification_bind(uint16_t src, uint64_t bitmap, uint32_t flags);
int  ffa_notification_get(uint16_t vcpu, uint32_t flags, uint64_t *bitmap);
int  ffa_notification_request(int notify_id, ffa_notifier_cb cb, void *data);
```

Data types (`struct ffa_send_direct_data`, `..._data2` (x4–x17),
`struct ffa_partition_info`, `struct ffa_mem_ops_args`,
`struct ffa_mem_region_addr_range`) are ported from the Linux header, adapted to
Zephyr types (`uint*_t`, `sys_dlist_t`, `k_mutex`, `k_sem`).

Gated by `CONFIG_ARM_FFA` (selects `ARM_SMCCC_1_2`), with sub-options
`CONFIG_ARM_FFA_MEM_SHARE` and `CONFIG_ARM_FFA_NOTIF`.

**What it does / interface / deps:** provides FF-A framework services to any
in-tree consumer via the header above. Depends on Component 1. Knows nothing
about OP-TEE.

### Component 3 — OP-TEE FF-A transport (`drivers/tee/optee/`)

Refactor the monolithic `optee.c` to isolate the conduit-specific parts behind a
transport interface, then add the FF-A transport. The `tee` driver API and the
`optee_msg` marshalling logic are shared and unchanged.

- `optee_transport.h` — `struct optee_transport_ops` with the calls the core
  driver needs from a conduit: `exchange_caps`, `get_os_revision`,
  `do_call_with_arg` (issue an OP-TEE RPC/yielding call and drive the RPC loop),
  `shm_register` / `shm_unregister`, `enable_async_notif`. Plus per-transport
  init.
- `optee_smc.c` — the **existing** SMC implementation moved here verbatim behind
  the ops. No behavioural change; the current `linaro,optee-tz` compat and
  `method` property keep working.
- `optee_ffa.c` — new FF-A transport. Ported from Linux `ffa_abi.c`:
  - discovery: find the OP-TEE partition via `ffa_partition_info_get` with
    OP-TEE's UUID; negotiate the OP-TEE protocol with
    `OPTEE_FFA_GET_API_VERSION` / `GET_OS_VERSION` /
    `EXCHANGE_CAPABILITIES`.
  - calls: `OPTEE_FFA_YIELDING_CALL_WITH_ARG` carried in a direct request; RPC
    return codes drive the same supplicant/RPC loop the SMC path uses.
  - shared memory: register via `ffa_mem_share`, pass the global handle to
    OP-TEE, reclaim via `ffa_mem_reclaim` + `OPTEE_FFA_UNREGISTER_SHM`.
  - async notifications: `OPTEE_FFA_ENABLE_ASYNC_NOTIF` + FF-A notifications wake
    the yielding-call waiters (replaces the SMC `handle_cmd_notify` path).
- `optee.c` — retains `tee` API entry points, `optee_msg` marshalling, the RPC
  command handlers, and supplicant queue; dispatches conduit work through
  `struct optee_transport_ops`.

**Devicetree:** new binding `dts/bindings/tee/linaro,optee-ffa.yaml` and compat
`linaro,optee-ffa` (conduit `method` = `smc`/`hvc`). The core driver selects the
transport from the matched compatible. A generic `arm,ffa` conduit node is *not*
required for OP-TEE (the partition is discovered at runtime); it may be added
later if a bus model is introduced.

**What it does / interface / deps:** presents the standard Zephyr `tee` API over
FF-A. Depends on Components 1–2 and the shared `optee_msg` code. The SMC path has
no new dependency.

## Data flow — invoking a TA over FF-A

1. Consumer calls `tee_invoke_func()` → OP-TEE driver builds an `optee_msg_arg`.
2. Buffer parameters are registered: `optee_ffa` calls `ffa_mem_share()` and
   substitutes the returned global handle into the message.
3. `optee_ffa.do_call_with_arg()` issues `OPTEE_FFA_YIELDING_CALL_WITH_ARG` via
   `ffa_msg_send_direct_req2()` → SMCCC 1.2 SMC → SPMD/SPMC → OP-TEE.
4. If OP-TEE returns an RPC/yield code, the shared RPC loop services it
   (supplicant, shm alloc/free, get-time) and resumes with
   `OPTEE_FFA_YIELDING_CALL_RESUME`.
5. On `RETURN_DONE`, results are unmarshalled back into `tee_param`s.
6. Buffers are released with `ffa_mem_reclaim()` + `OPTEE_FFA_UNREGISTER_SHM`.
7. Async completions (when negotiated) arrive as FF-A notifications that wake the
   waiting thread instead of a synchronous poll.

## Error handling

- **Central mapping:** one `ffa_to_errno()` converting `FFA_ERROR` codes
  (`NOT_SUPPORTED`, `INVALID_PARAMETERS`, `NO_MEMORY`, `BUSY`, `RETRY`,
  `DENIED`, `ABORTED`, `NO_DATA`) to `-errno`. `BUSY`/`RETRY` are retried with a
  bounded backoff at the call site.
- **Version negotiation:** offer 1.2; if the SPMC returns lower, store the
  negotiated version and feature-gate at runtime (e.g. fall back
  `DIRECT_REQ2`→`DIRECT_REQ`, `PARTITION_INFO_GET_REGS`→RX-buffer variant). If
  the SPMC reports `NOT_SUPPORTED` for `FFA_VERSION`, FF-A init fails cleanly.
- **Transport isolation:** FF-A transport init failure (no OP-TEE partition, ABI
  mismatch) fails only the FF-A device probe; the SMC path is unaffected.
- **Buffer lifetime:** every `ffa_mem_share` has a matched reclaim on all exit
  paths, including error unwinding, to avoid leaking secure-world grants.

## Testing / verification

- **Unit tests** (`tests/subsys/firmware/ffa/`, host/native where feasible):
  - register marshalling for `arm_smccc_1_2_regs` (mocked conduit).
  - `FFA_VERSION` negotiation + downgrade logic.
  - composite memory-region descriptor construction (single + fragmented).
  - `ffa_to_errno()` mapping table.
- **Integration smoke test** on `qemu_cortex_a53` with an OP-TEE build acting as
  SP/SPMC: probe OP-TEE over FF-A, open a session to the built-in pseudo-TA,
  invoke a command with a shared-memory buffer, close. Reuse/extend
  `tests/drivers/tee` gated on the FF-A compat.
- **Regression:** existing SMC OP-TEE path still builds and passes on its board.
- **Docs:** subsystem doc page + a `release-notes` entry.

## Implementation phases (one spec, phased delivery)

Each phase is independently buildable, reviewable, and mergeable. Later phases
depend on earlier ones.

- **SP-1 — SMCCC 1.2 call layer.** `arm_smccc_1_2_regs` + asm + `CONFIG_ARM_SMCCC_1_2`.
  Unit test with a trivial known SMCCC call (e.g. `FFA_VERSION` or SMCCC version).
- **SP-2 — FF-A core: setup + discovery + direct messaging.** `ffa_core.c`,
  `ffa_msg.c`, public header, `CONFIG_ARM_FFA`. Version negotiation, RXTX map,
  partition info, direct req/resp. Unit tests for negotiation + marshalling.
- **SP-3 — FF-A core: memory sharing.** `ffa_mem.c`, `CONFIG_ARM_FFA_MEM_SHARE`.
  Descriptor construction + fragmentation + reclaim. Unit tests.
- **SP-4 — FF-A core: notifications.** `ffa_notif.c`, `CONFIG_ARM_FFA_NOTIF`.
  Bitmap/bind/set/get/info + schedule-receiver handling.
- **SP-5 — OP-TEE FF-A transport + verification.** Transport-ops refactor
  (`optee_transport.h`, `optee_smc.c` extraction), `optee_ffa.c`, DT binding +
  `linaro,optee-ffa` compat, `qemu_cortex_a53` + OP-TEE bring-up, smoke test,
  docs, release note.

## Rejected alternatives

- **Full Linux-style FF-A bus** (`ffa_device`/`ffa_driver`, partition-UUID
  driver matching): faithful to Linux but doesn't map cleanly onto Zephyr's
  device model, and there is exactly one consumer today. A flat function API is
  simpler and sufficient; a bus can be added later without breaking the API.
- **Minimal fold-in** (FF-A stuffed into `optee.c` behind the method switch):
  smallest, but not reusable — contradicts the "FF-A supported as a subsystem"
  goal.
- **Extending the existing 8-register `arm_smccc_res`** to 18: would churn PSCI
  and the SMC OP-TEE callers for no benefit. A separate SMCCC 1.2 struct/entry
  point is cleaner and matches Linux's `arm_smccc_1_2_smc`.

## Open questions / risks

- **QEMU + OP-TEE FF-A topology:** exact `qemu_cortex_a53` + OP-TEE (SPMD-at-EL3
  vs Hafnium SPMC) configuration to be pinned down in SP-5; the NS-endpoint code
  is topology-agnostic but the test harness must boot a known-good secure stack.
- **OP-TEE protocol vs FF-A framework version skew:** OP-TEE advertises OP-TEE
  FF-A protocol 1.0 while the framework may be 1.2; both negotiations are kept
  independent to avoid coupling.
- **Notifications complexity (SP-4):** highest-risk/most-optional. If schedule-
  receiver interrupt wiring proves board-specific on QEMU, fall back to the
  poll-based completion the SMC path already uses; async notif is an
  optimization, not a correctness requirement.
