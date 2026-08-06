# FF-A SP-2b: partition discovery + direct messaging — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the FF-A core subsystem (from SP-2a) with partition discovery (`FFA_PARTITION_INFO_GET` + the register variant `FFA_PARTITION_INFO_GET_REGS`) and direct messaging (`FFA_MSG_SEND_DIRECT_REQ`/`RESP` and the 1.2 `REQ2`/`RESP2`), exposing the flat public API a consumer (OP-TEE transport, SP-5) will call.

**Architecture:** A new file `subsys/firmware/ffa/ffa_msg.c` sits beside `ffa_core.c` in the same `zephyr_library`, reusing everything SP-2a built: the `ffa_invoke()` conduit seam, `ffa_to_errno()`, the negotiated `st->version`, the RX buffer, and `st->lock`. Partition discovery reads partition records either from the shared RX buffer (`FFA_PARTITION_INFO_GET`, all versions) or directly from return registers (`FFA_PARTITION_INFO_GET_REGS`, FF-A ≥ 1.2) — the register variant is preferred when features advertise it, with a runtime fallback to the RX-buffer variant. Direct messaging marshals the caller's payload into x3–x7 (`REQ`) or x4–x17 (`REQ2`, which also carries the target UUID in x2–x3), drives the `FFA_INTERRUPT`/`FFA_YIELD`→`FFA_RUN` completion loop, and unpacks the `RESP`/`RESP2` payload back. All logic is unit-tested against the SP-2a mock conduit — no live responder.

**Tech Stack:** Zephyr (AArch64), C, Kconfig, CMake, Ztest, `west`/twister on `qemu_cortex_a53`. Build+test on fio via `.tools/fio-push` + `.tools/fio-run` (see `WORKFLOW.md`); authoring is local.

## Global Constraints

- **Builds on SP-2a (realized, in tree).** Reuse verbatim — do not redefine:
  - `subsys/firmware/ffa/ffa_internal.h`: FID macros (`FFA_ERROR=0x84000060`, `FFA_SUCCESS_32=0x84000061`, `FFA_SUCCESS_64=0xC4000061`, `FFA_INTERRUPT=0x84000062`, `FFA_VERSION`, `FFA_FEATURES`, `FFA_RX_RELEASE=0x84000065`, `FFA_PARTITION_INFO_GET=0x84000068`, `FFA_ID_GET`), `FFA_RET_*` codes, `FFA_PACK_VERSION`/`FFA_VERSION_MAJOR`/`FFA_VERSION_MINOR`, `FFA_VERSION_1_0/1_1/1_2`, `struct ffa_drv_state`, `ffa_conduit_fn_t`, and the prototypes `ffa_invoke`, `ffa_to_errno`, `ffa_negotiate_version`, `ffa_get_id`, `ffa_query_feature`, `ffa_rxtx_map`, `ffa_test_set_conduit`.
  - `include/zephyr/firmware/ffa.h`: `struct ffa_uuid { uint8_t bytes[16]; }`, `ffa_is_available`, `ffa_version`, `ffa_id_get`, and the Doxygen group `ffa_core_api` (add new public decls INSIDE the `@{`/`@}` fence).
  - `struct ffa_drv_state` fields available: `bool available; uint32_t version; uint16_t vm_id; enum arm_smccc_conduit conduit; void *tx_buf; void *rx_buf; uint32_t rxtx_pages; struct k_mutex lock;`.
- **Canonical FF-A FIDs to ADD** (SMC fast-call owner=Standard; 32-bit `0x84000000|fn`, 64-bit `0xC4000000|fn`; values verified against `optee_os/core/arch/arm/include/ffa.h`):
  `FFA_RUN=0x8400006D`, `FFA_YIELD=0x8400006C`, `FFA_MSG_SEND_DIRECT_REQ_32=0x8400006F`, `FFA_MSG_SEND_DIRECT_REQ_64=0xC400006F`, `FFA_MSG_SEND_DIRECT_RESP_32=0x84000070`, `FFA_MSG_SEND_DIRECT_RESP_64=0xC4000070`, `FFA_MSG_SEND_DIRECT_REQ2=0xC400008D`, `FFA_MSG_SEND_DIRECT_RESP2=0xC400008E`, `FFA_PARTITION_INFO_GET_REGS=0xC400008B`.
- **Endpoint-ID packing:** `src_dst_ids = (src_id << 16) | dst_id` (sender in bits[31:16], receiver in bits[15:0]).
- **`PARTITION_INFO_GET` register convention (RX-buffer variant):** a0=`FFA_PARTITION_INFO_GET`, a1..a4 = the four 32-bit UUID words (a1=uuid0…a4=uuid3), a5=flags. On success a0=`FFA_SUCCESS_32`, a2=partition count, a3=record size (only when version > 1.0; else record size is fixed 8). Records live in the RX buffer; each record (v>1.0) is `{u16 id; u16 exec_ctxt; u32 properties; uint8_t uuid[16];}`; the v1.0 record is the first 8 bytes only (no uuid). After copying, call `FFA_RX_RELEASE` (unless count-only). `flags` bit0 = "return count only" (only when version > 1.0). All fields little-endian.
- **`PARTITION_INFO_GET_REGS` convention (register variant, v≥1.2):** a0=`FFA_PARTITION_INFO_GET_REGS`, a1=`((u64)uuid1<<32)|uuid0`, a2=`((u64)uuid3<<32)|uuid2`, a3=`start_idx | (tag<<16)`. On success a0=`FFA_SUCCESS_64`(or `FFA_SUCCESS_32`), a2 packs `count=(bits[15:0])+1`, `cur_idx=bits[31:16]`, `tag=bits[47:32]`, `part_info_sz=bits[63:48]`; partition records are packed into a3.. (each record = 3×u64: word0 = `id=bits[15:0] | exec_ctxt=bits[31:16] | properties=bits[63:32]`, words1-2 = 16-byte uuid). Iterate with `start_idx`/`cur_idx` until `cur_idx >= count-1`.
- **`MSG_SEND_DIRECT_REQ` convention:** pick `REQ_32`/`RESP_32` if the target is 32-bit mode, else native 64-bit (`REQ_64`/`RESP_64`). a0=req_id, a1=src_dst_ids, a2=0, a3..a7 = data0..data4. Drive completion loop: while a0==`FFA_INTERRUPT`||`FFA_YIELD`, re-invoke `FFA_RUN` with a1=ret.a1. On a0==`FFA_ERROR` → `ffa_to_errno(a2)`. On a0==resp_id → copy a3..a7 back into data0..data4, return 0. Else -EINVAL.
- **`MSG_SEND_DIRECT_REQ2` convention (v1.2):** a0=`FFA_MSG_SEND_DIRECT_REQ2`, a1=src_dst_ids, a2=uuid low 8 bytes (LE), a3=uuid high 8 bytes (LE), a4..a17 = data[0..13]. Same completion loop. On a0==`FFA_MSG_SEND_DIRECT_RESP2` → copy a4..a17 back into data[0..13], return 0. On `FFA_ERROR` → `ffa_to_errno(a2)`. Else -EINVAL.
- **UUID byte order:** `struct ffa_uuid.bytes[16]` is the raw 16-byte UUID as the FF-A spec transports it. For REQ2, a2 = little-endian u64 of bytes[0..7], a3 = LE u64 of bytes[8..15]. Provide a small helper to read/write these without unaligned traps (use `memcpy` into a `uint64_t`, no pointer casts through unaligned addresses).
- **RX-buffer serialization:** partition discovery that uses the RX buffer MUST hold `st->lock` across the invoke + record copy + `FFA_RX_RELEASE`, because the RX buffer is a shared singleton resource. The register variant does not touch the RX buffer and needs no lock.
- **Version gating:** `ffa_partition_info_get` uses the `_REGS` variant iff `st->version >= FFA_VERSION_1_2` AND `ffa_query_feature(FFA_PARTITION_INFO_GET_REGS)` returns 0; otherwise the RX-buffer variant. `ffa_msg_send_direct_req2` requires `st->version >= FFA_VERSION_1_2`; if lower, return `-ENOTSUP`.
- **Testing:** `qemu_cortex_a53` has no FF-A responder. Every function is unit-tested by installing a mock conduit (`ffa_test_set_conduit`) that captures request registers and returns canned responses, asserting BOTH marshalling and unpacking. For RX-buffer partition tests, the test points `st->rx_buf` at a local buffer it pre-populates with fake records. No live SMC.
- **License:** Apache-2.0, `Copyright 2026 Qualcomm Innovation Center, Inc.` on new files.
- **Commits:** `git commit --no-gpg-sign` + trailer `Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>`. No `Co-Authored-By`. Branch `ffa-optee-support`.
- **Scope:** discovery + direct messaging only. Memory sharing (SP-3), notifications (SP-4), and the OP-TEE transport (SP-5) are out of scope.

---

## File Structure

- `subsys/firmware/ffa/ffa_internal.h` (modify) — add the new FIDs, the partition-info/target-info bit-field macros, `PACK_TARGET_INFO`, and the internal prototypes for the new calls (taking `struct ffa_drv_state *`).
- `include/zephyr/firmware/ffa.h` (modify) — add public types `struct ffa_partition_info`, `struct ffa_send_direct_data`, `struct ffa_send_direct_data2`, and the public API `ffa_partition_info_get`, `ffa_msg_send_direct_req`, `ffa_msg_send_direct_req2` (inside the `ffa_core_api` group fence).
- `subsys/firmware/ffa/ffa_msg.c` (create) — implementations. Includes `ffa_internal.h` + the public header.
- `subsys/firmware/ffa/CMakeLists.txt` (modify) — add `ffa_msg.c` to `zephyr_library_sources`.
- `tests/subsys/firmware/ffa/src/main.c` (modify) — add mock-conduit tests for each new call (same suite `ffa_core`).

---

### Task 1: ABI constants, macros, and public types

**Files:**
- Modify: `subsys/firmware/ffa/ffa_internal.h`
- Modify: `include/zephyr/firmware/ffa.h`

**Interfaces:**
- Consumes: SP-2a header contents (do not redefine).
- Produces: the new FID macros, the field-extraction macros, `PACK_TARGET_INFO`, and public structs/prototypes listed below.

- [ ] **Step 1: Add FIDs + macros to ffa_internal.h**

Insert into the FID block (after `FFA_ID_GET`):

```c
#define FFA_YIELD                     0x8400006CU
#define FFA_RUN                       0x8400006DU
#define FFA_MSG_SEND_DIRECT_REQ_32    0x8400006FU
#define FFA_MSG_SEND_DIRECT_REQ_64    0xC400006FU
#define FFA_MSG_SEND_DIRECT_RESP_32   0x84000070U
#define FFA_MSG_SEND_DIRECT_RESP_64   0xC4000070U
#define FFA_MSG_SEND_DIRECT_REQ2      0xC400008DU
#define FFA_MSG_SEND_DIRECT_RESP2     0xC400008EU
#define FFA_PARTITION_INFO_GET_REGS   0xC400008BU
```

Add the field macros (near the version macros):

```c
/* Endpoint pair packing for direct messages: sender[31:16], receiver[15:0]. */
#define FFA_PACK_TARGET_INFO(src, dst) \
	((((uint32_t)(src) & 0xFFFFU) << 16) | ((uint32_t)(dst) & 0xFFFFU))

/* PARTITION_INFO_GET flags. */
#define FFA_PARTITION_INFO_GET_COUNT_ONLY  0x1U

/* FF-A 1.0 partition record is the first 8 bytes (no UUID). */
#define FFA_1_0_PARTITION_INFO_SZ          8U

/* PARTITION_INFO_GET_REGS: packed fields in a2. */
#define FFA_PIG_REGS_LAST_IDX(x)   ((uint16_t)((x) & 0xFFFFU))
#define FFA_PIG_REGS_CUR_IDX(x)    ((uint16_t)(((x) >> 16) & 0xFFFFU))
#define FFA_PIG_REGS_TAG(x)        ((uint16_t)(((x) >> 32) & 0xFFFFU))
#define FFA_PIG_REGS_SIZE(x)       ((uint16_t)(((x) >> 48) & 0xFFFFU))
/* PARTITION_INFO_GET_REGS: packed partition record word0. */
#define FFA_PIG_REC_ID(x)          ((uint16_t)((x) & 0xFFFFU))
#define FFA_PIG_REC_EXEC_CTXT(x)   ((uint16_t)(((x) >> 16) & 0xFFFFU))
#define FFA_PIG_REC_PROPS(x)       ((uint32_t)(((x) >> 32) & 0xFFFFFFFFU))
```

- [ ] **Step 2: Add public types + API to include/zephyr/firmware/ffa.h**

Inside the `@{ ... @}` group fence, after the existing accessors, add:

```c
/** FF-A partition properties (subset relevant to a NS endpoint). */
#define FFA_PARTITION_DIRECT_RECV        BIT(0)
#define FFA_PARTITION_DIRECT_SEND        BIT(1)
#define FFA_PARTITION_INDIRECT_MSG       BIT(2)
#define FFA_PARTITION_NOTIFICATION_RECV  BIT(3)
#define FFA_PARTITION_AARCH64_EXEC       BIT(8)
#define FFA_PARTITION_DIRECT_REQ2_RECV   BIT(9)
#define FFA_PARTITION_DIRECT_REQ2_SEND   BIT(10)

/** One partition descriptor returned by ffa_partition_info_get(). */
struct ffa_partition_info {
	uint16_t id;            /**< partition/endpoint ID */
	uint16_t exec_ctxt;     /**< number of execution contexts */
	uint32_t properties;    /**< FFA_PARTITION_* property bits */
	struct ffa_uuid uuid;   /**< partition UUID (zero if negotiated version is 1.0) */
};

/** Register payload for FFA_MSG_SEND_DIRECT_REQ/RESP (x3-x7). */
struct ffa_send_direct_data {
	unsigned long data0;
	unsigned long data1;
	unsigned long data2;
	unsigned long data3;
	unsigned long data4;
};

/** Register payload for FFA_MSG_SEND_DIRECT_REQ2/RESP2 (x4-x17). */
struct ffa_send_direct_data2 {
	unsigned long data[14];
};

/**
 * @brief Discover partitions matching a UUID.
 * @param uuid  UUID to match; pass the nil UUID (all-zero) to enumerate all.
 * @param out   caller buffer of @p max entries (may be NULL to count only).
 * @param count in: capacity of @p out; out: number of partitions reported.
 * @retval 0 on success, negative errno otherwise, -EAGAIN if FF-A unavailable.
 */
int ffa_partition_info_get(const struct ffa_uuid *uuid,
			   struct ffa_partition_info *out, size_t *count);

/**
 * @brief Send a direct request (FFA_MSG_SEND_DIRECT_REQ) and await the response.
 * @param dst   destination endpoint ID.
 * @param data  in/out register payload (x3-x7).
 * @retval 0 on success (response unpacked into @p data), negative errno otherwise.
 */
int ffa_msg_send_direct_req(uint16_t dst, struct ffa_send_direct_data *data);

/**
 * @brief Send a direct request2 (FFA_MSG_SEND_DIRECT_REQ2, FF-A >= 1.2).
 * @param dst   destination endpoint ID.
 * @param uuid  target service UUID.
 * @param data  in/out register payload (x4-x17).
 * @retval 0 on success, -ENOTSUP if negotiated version < 1.2, negative errno otherwise.
 */
int ffa_msg_send_direct_req2(uint16_t dst, const struct ffa_uuid *uuid,
			     struct ffa_send_direct_data2 *data);
```

Ensure `#include <zephyr/sys/util.h>` (for `BIT`) is present in the public header, or use explicit `(1U << n)` if the header must stay dependency-free — CHECK what SP-2a's header already includes and match it (prefer explicit shifts if it currently includes only `<stdint.h>`/`<stdbool.h>`).

- [ ] **Step 3: Build the header change (no impl yet)**

```bash
git add subsys/firmware/ffa/ffa_internal.h include/zephyr/firmware/ffa.h
git commit --no-gpg-sign -m "subsys: firmware: ffa: ABI + public types for discovery and messaging

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
/home/jramirez/Work/zephyr/.tools/fio-push
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west twister -p qemu_cortex_a53 -T tests/subsys/firmware/ffa'
```
Expected: PASS — existing 13 tests still build/run (new decls are unused so far; header compiles).

- [ ] **Step 4: Commit** (done in Step 3).

---

### Task 2: `ffa_msg_send_direct_req` + completion loop (TDD)

**Files:**
- Create: `subsys/firmware/ffa/ffa_msg.c`
- Modify: `subsys/firmware/ffa/CMakeLists.txt`
- Modify: `subsys/firmware/ffa/ffa_internal.h` (add internal prototype)
- Modify: `tests/subsys/firmware/ffa/src/main.c`

**Interfaces:**
- Consumes: `ffa_invoke`, `ffa_to_errno`, FIDs, `FFA_PACK_TARGET_INFO`, `struct ffa_send_direct_data` (Task 1).
- Produces: internal `int ffa_send_direct_req(struct ffa_drv_state *st, uint16_t dst, bool mode_32bit, struct ffa_send_direct_data *data);` and a shared `static void ffa_msg_wait_for_completion(struct ffa_drv_state *st, struct arm_smccc_1_2_regs *ret);`. The public `ffa_msg_send_direct_req` (Task 5) wraps the internal one on the singleton.

- [ ] **Step 1: Add the CMake source + internal prototype**

`CMakeLists.txt`: add `ffa_msg.c` to `zephyr_library_sources` (now `ffa_core.c ffa_msg.c`).

`ffa_internal.h` (append inside the guard):
```c
int ffa_send_direct_req(struct ffa_drv_state *st, uint16_t dst, bool mode_32bit,
			struct ffa_send_direct_data *data);
```

- [ ] **Step 2: Write failing tests**

Add to `tests/subsys/firmware/ffa/src/main.c` (mock harness from SP-2a: `mock_conduit`, `mock_last_args`, `mock_next_res`; add a small multi-response queue helper if needed for the completion loop — see below).

First add a tiny scripted-response capability so a test can return `FFA_INTERRUPT` then the real response:

```c
/* Scripted responses for multi-invoke sequences (e.g. INTERRUPT -> RUN -> RESP). */
static struct arm_smccc_1_2_regs mock_script[4];
static int mock_script_len;
static int mock_script_pos;
static struct arm_smccc_1_2_regs mock_seen[4];
static int mock_seen_n;

static void mock_script_conduit(const struct arm_smccc_1_2_regs *args,
				struct arm_smccc_1_2_regs *res)
{
	if (mock_seen_n < (int)ARRAY_SIZE(mock_seen)) {
		mock_seen[mock_seen_n++] = *args;
	}
	if (mock_script_pos < mock_script_len) {
		*res = mock_script[mock_script_pos++];
	} else {
		memset(res, 0, sizeof(*res));
	}
}

static void mock_script_reset(void)
{
	mock_script_len = 0;
	mock_script_pos = 0;
	mock_seen_n = 0;
	memset(mock_script, 0, sizeof(mock_script));
	memset(mock_seen, 0, sizeof(mock_seen));
}
```

Tests:

```c
ZTEST(ffa_core, test_direct_req_roundtrip_64)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {
		.data0 = 0x11, .data1 = 0x22, .data2 = 0x33,
		.data3 = 0x44, .data4 = 0x55,
	};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* single response: RESP_64 with echoed+1 payload */
	mock_script[0] = (struct arm_smccc_1_2_regs){
		.a0 = FFA_MSG_SEND_DIRECT_RESP_64,
		.a3 = 0x111, .a4 = 0x222, .a5 = 0x333, .a6 = 0x444, .a7 = 0x555,
	};
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req(&st, 0x8001, false, &d), 0, NULL);
	/* request marshalling */
	zassert_equal(mock_seen[0].a0, FFA_MSG_SEND_DIRECT_REQ_64, NULL);
	zassert_equal(mock_seen[0].a1, FFA_PACK_TARGET_INFO(st.vm_id, 0x8001), NULL);
	zassert_equal(mock_seen[0].a2, 0, NULL);
	zassert_equal(mock_seen[0].a3, 0x11, NULL);
	zassert_equal(mock_seen[0].a7, 0x55, NULL);
	/* response unpacked */
	zassert_equal(d.data0, 0x111, NULL);
	zassert_equal(d.data4, 0x555, NULL);
}

ZTEST(ffa_core, test_direct_req_32bit_fid)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_MSG_SEND_DIRECT_RESP_32 };
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req(&st, 0x1, true, &d), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_MSG_SEND_DIRECT_REQ_32, NULL);
}

ZTEST(ffa_core, test_direct_req_completion_loop)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* INTERRUPT (a1 carries the target for FFA_RUN) then real RESP */
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_INTERRUPT, .a1 = 0xABCD };
	mock_script[1] = (struct arm_smccc_1_2_regs){ .a0 = FFA_MSG_SEND_DIRECT_RESP_64 };
	mock_script_len = 2;

	zassert_equal(ffa_send_direct_req(&st, 0x2, false, &d), 0, NULL);
	/* second invoke must be FFA_RUN with a1 from the interrupt */
	zassert_equal(mock_seen[1].a0, FFA_RUN, NULL);
	zassert_equal(mock_seen[1].a1, 0xABCD, NULL);
}

ZTEST(ffa_core, test_direct_req_error)
{
	struct ffa_drv_state st = {0};
	struct ffa_send_direct_data d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_ERROR, .a2 = (unsigned long)FFA_RET_BUSY };
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req(&st, 0x3, false, &d), -EBUSY, NULL);
}
```

Run twister → CONFIRM these FAIL (undefined `ffa_send_direct_req`).

- [ ] **Step 3: Implement ffa_msg.c**

```c
/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/firmware/ffa.h>
#include "ffa_internal.h"

LOG_MODULE_DECLARE(arm_ffa, CONFIG_ARM_FFA_LOG_LEVEL);

static void ffa_msg_wait_for_completion(struct ffa_drv_state *st,
					struct arm_smccc_1_2_regs *ret)
{
	while ((uint32_t)ret->a0 == FFA_INTERRUPT ||
	       (uint32_t)ret->a0 == FFA_YIELD) {
		struct arm_smccc_1_2_regs args = {0};

		if ((uint32_t)ret->a0 == FFA_YIELD) {
			k_msleep(1);
		}
		args.a0 = FFA_RUN;
		args.a1 = ret->a1;
		ffa_invoke(st, &args, ret);
	}
}

int ffa_send_direct_req(struct ffa_drv_state *st, uint16_t dst, bool mode_32bit,
			struct ffa_send_direct_data *data)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};
	uint32_t req_id = mode_32bit ? FFA_MSG_SEND_DIRECT_REQ_32
				     : FFA_MSG_SEND_DIRECT_REQ_64;
	uint32_t resp_id = mode_32bit ? FFA_MSG_SEND_DIRECT_RESP_32
				      : FFA_MSG_SEND_DIRECT_RESP_64;

	args.a0 = req_id;
	args.a1 = FFA_PACK_TARGET_INFO(st->vm_id, dst);
	args.a2 = 0;
	args.a3 = data->data0;
	args.a4 = data->data1;
	args.a5 = data->data2;
	args.a6 = data->data3;
	args.a7 = data->data4;

	ffa_invoke(st, &args, &ret);
	ffa_msg_wait_for_completion(st, &ret);

	if ((uint32_t)ret.a0 == FFA_ERROR) {
		return ffa_to_errno((int)ret.a2);
	}
	if ((uint32_t)ret.a0 == resp_id) {
		data->data0 = ret.a3;
		data->data1 = ret.a4;
		data->data2 = ret.a5;
		data->data3 = ret.a6;
		data->data4 = ret.a7;
		return 0;
	}
	return -EINVAL;
}
```

Run twister → CONFIRM all pass (13 + 4 = 17). Commit:

```bash
git add subsys/firmware/ffa/ffa_msg.c subsys/firmware/ffa/CMakeLists.txt subsys/firmware/ffa/ffa_internal.h tests/subsys/firmware/ffa/src/main.c
git commit --no-gpg-sign -m "subsys: firmware: ffa: direct request messaging (REQ/RESP)

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 3: `ffa_msg_send_direct_req2` (TDD)

**Files:**
- Modify: `subsys/firmware/ffa/ffa_msg.c`, `ffa_internal.h`, `tests/.../main.c`

**Interfaces:**
- Consumes: Task 2 completion loop, `struct ffa_send_direct_data2`, `struct ffa_uuid`.
- Produces: internal `int ffa_send_direct_req2(struct ffa_drv_state *st, uint16_t dst, const struct ffa_uuid *uuid, struct ffa_send_direct_data2 *data);` (public wrapper in Task 5).

- [ ] **Step 1: Add prototype + a UUID→u64 helper**

`ffa_internal.h`:
```c
int ffa_send_direct_req2(struct ffa_drv_state *st, uint16_t dst,
			 const struct ffa_uuid *uuid,
			 struct ffa_send_direct_data2 *data);
```

- [ ] **Step 2: Write failing tests** (add to main.c):

```c
ZTEST(ffa_core, test_direct_req2_roundtrip)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_2 };
	struct ffa_uuid uuid = { .bytes = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15} };
	struct ffa_send_direct_data2 d = {0};

	for (int i = 0; i < 14; i++) {
		d.data[i] = 0x100 + i;
	}
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_MSG_SEND_DIRECT_RESP2;
	for (int i = 0; i < 14; i++) {
		((unsigned long *)&mock_script[0].a4)[i] = 0x200 + i; /* see note */
	}
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req2(&st, 0x8001, &uuid, &d), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_MSG_SEND_DIRECT_REQ2, NULL);
	zassert_equal(mock_seen[0].a1, FFA_PACK_TARGET_INFO(st.vm_id, 0x8001), NULL);
	/* a2 = LE u64 of bytes[0..7], a3 = LE u64 of bytes[8..15] */
	zassert_equal(mock_seen[0].a2, 0x0706050403020100UL, NULL);
	zassert_equal(mock_seen[0].a3, 0x0F0E0D0C0B0A0908UL, NULL);
	/* payload x4.. carried data[0..13] */
	zassert_equal(mock_seen[0].a4, 0x100, NULL);
	zassert_equal(mock_seen[0].a17, 0x100 + 13, NULL);
	/* response unpacked */
	zassert_equal(d.data[0], 0x200, NULL);
	zassert_equal(d.data[13], 0x200 + 13, NULL);
}

ZTEST(ffa_core, test_direct_req2_requires_1_2)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1 };
	struct ffa_uuid uuid = {0};
	struct ffa_send_direct_data2 d = {0};

	ffa_test_set_conduit(mock_script_conduit); /* must not be invoked */
	zassert_equal(ffa_send_direct_req2(&st, 0x1, &uuid, &d), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_direct_req2_error)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_2 };
	struct ffa_uuid uuid = {0};
	struct ffa_send_direct_data2 d = {0};

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0].a0 = FFA_ERROR;
	mock_script[0].a2 = (unsigned long)FFA_RET_DENIED;
	mock_script_len = 1;

	zassert_equal(ffa_send_direct_req2(&st, 0x1, &uuid, &d), -EACCES, NULL);
}
```

> Note on the test: since `struct arm_smccc_1_2_regs` a0..a17 are contiguous `unsigned long`, indexing `&a4` as an array is valid given the SP-1 layout `BUILD_ASSERT`. If the reviewer prefers, set fields explicitly (a4..a17). Keep whichever is clearer; both rely on the same guaranteed layout.

Run twister → CONFIRM fail.

- [ ] **Step 3: Implement in ffa_msg.c**

```c
/* Read a little-endian u64 from a byte array without unaligned pointer casts. */
static uint64_t ffa_uuid_lo(const struct ffa_uuid *u)
{
	uint64_t v;

	memcpy(&v, &u->bytes[0], sizeof(v));
	return v; /* AArch64 is little-endian; bytes[0] is the LSB */
}

static uint64_t ffa_uuid_hi(const struct ffa_uuid *u)
{
	uint64_t v;

	memcpy(&v, &u->bytes[8], sizeof(v));
	return v;
}

int ffa_send_direct_req2(struct ffa_drv_state *st, uint16_t dst,
			 const struct ffa_uuid *uuid,
			 struct ffa_send_direct_data2 *data)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};
	unsigned long *argp = &args.a4;
	unsigned long *retp;

	if (st->version < FFA_VERSION_1_2) {
		return -ENOTSUP;
	}

	args.a0 = FFA_MSG_SEND_DIRECT_REQ2;
	args.a1 = FFA_PACK_TARGET_INFO(st->vm_id, dst);
	args.a2 = ffa_uuid_lo(uuid);
	args.a3 = ffa_uuid_hi(uuid);
	for (int i = 0; i < 14; i++) {
		argp[i] = data->data[i];
	}

	ffa_invoke(st, &args, &ret);
	ffa_msg_wait_for_completion(st, &ret);

	if ((uint32_t)ret.a0 == FFA_ERROR) {
		return ffa_to_errno((int)ret.a2);
	}
	if ((uint32_t)ret.a0 == FFA_MSG_SEND_DIRECT_RESP2) {
		retp = &ret.a4;
		for (int i = 0; i < 14; i++) {
			data->data[i] = retp[i];
		}
		return 0;
	}
	return -EINVAL;
}
```

The `&args.a4` array walk relies on the SP-1 layout guarantee (18 contiguous longs). Add a `BUILD_ASSERT` in ffa_msg.c mirroring ffa_core.c's, OR add a localized comment referencing it. Prefer adding:
```c
BUILD_ASSERT(offsetof(struct arm_smccc_1_2_regs, a17) ==
	     offsetof(struct arm_smccc_1_2_regs, a4) + 13 * sizeof(unsigned long),
	     "a4..a17 must be 14 contiguous longs for REQ2 payload marshalling");
```

Run twister → CONFIRM pass (17 + 3 = 20). Commit:
```bash
git commit --no-gpg-sign -m "subsys: firmware: ffa: direct request2 messaging (REQ2/RESP2)

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 4: `ffa_partition_info_get` — RX-buffer variant (TDD)

**Files:**
- Modify: `subsys/firmware/ffa/ffa_msg.c`, `ffa_internal.h`, `tests/.../main.c`

**Interfaces:**
- Consumes: `ffa_invoke`, `ffa_to_errno`, `st->rx_buf`, `st->lock`, `st->version`, `struct ffa_partition_info`.
- Produces: internal `int ffa_partition_info_get_rxbuf(struct ffa_drv_state *st, const struct ffa_uuid *uuid, struct ffa_partition_info *out, size_t *count);` and an internal `int ffa_rx_release(struct ffa_drv_state *st);`.

- [ ] **Step 1: Prototypes** in ffa_internal.h:
```c
int ffa_rx_release(struct ffa_drv_state *st);
int ffa_partition_info_get_rxbuf(struct ffa_drv_state *st,
				 const struct ffa_uuid *uuid,
				 struct ffa_partition_info *out, size_t *count);
```

- [ ] **Step 2: Write failing tests.** The test installs the mock, points `st->rx_buf` at a local buffer pre-filled with fake v>1.0 records, and returns a `FFA_SUCCESS_32` with count in a2 and record-size in a3. Verify: UUID words marshalled into a1..a4; count returned; records copied+decoded; `FFA_RX_RELEASE` issued as the 2nd invoke. Also a count-only test (out=NULL) and an error test.

```c
/* On-wire v>1.0 partition record layout. */
struct pinfo_rec_le { uint16_t id; uint16_t exec; uint32_t props; uint8_t uuid[16]; };

ZTEST(ffa_core, test_partition_info_rxbuf)
{
	static uint8_t rxbuf[256] __aligned(8);
	struct pinfo_rec_le *rec = (struct pinfo_rec_le *)rxbuf;
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1, .rx_buf = rxbuf };
	struct ffa_uuid uuid = {0};
	struct ffa_partition_info out[2];
	size_t count = 2;

	k_mutex_init(&st.lock);
	rec[0].id = 0x8001; rec[0].exec = 1; rec[0].props = 0x3;
	for (int i = 0; i < 16; i++) { rec[0].uuid[i] = i; }

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){
		.a0 = FFA_SUCCESS_32, .a2 = 1, .a3 = sizeof(struct pinfo_rec_le),
	};
	mock_script[1] = (struct arm_smccc_1_2_regs){ .a0 = FFA_SUCCESS_32 }; /* RX_RELEASE */
	mock_script_len = 2;

	zassert_equal(ffa_partition_info_get_rxbuf(&st, &uuid, out, &count), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_PARTITION_INFO_GET, NULL);
	zassert_equal(count, 1, NULL);
	zassert_equal(out[0].id, 0x8001, NULL);
	zassert_equal(out[0].properties, 0x3, NULL);
	zassert_equal(out[0].uuid.bytes[15], 15, NULL);
	zassert_equal(mock_seen[1].a0, FFA_RX_RELEASE, NULL); /* released */
}

ZTEST(ffa_core, test_partition_info_count_only)
{
	static uint8_t rxbuf[64] __aligned(8);
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1, .rx_buf = rxbuf };
	struct ffa_uuid uuid = {0};
	size_t count = 0;

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_SUCCESS_32, .a2 = 3 };
	mock_script_len = 1;

	zassert_equal(ffa_partition_info_get_rxbuf(&st, &uuid, NULL, &count), 0, NULL);
	zassert_equal(count, 3, NULL);
	/* count-only sets flags bit0 and does NOT release RX */
	zassert_equal(mock_seen[0].a5, FFA_PARTITION_INFO_GET_COUNT_ONLY, NULL);
}

ZTEST(ffa_core, test_partition_info_error)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_1 };
	struct ffa_uuid uuid = {0};
	size_t count = 0;

	k_mutex_init(&st.lock);
	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	mock_script[0] = (struct arm_smccc_1_2_regs){ .a0 = FFA_ERROR, .a2 = (unsigned long)FFA_RET_INVALID_PARAMETERS };
	mock_script_len = 1;

	zassert_equal(ffa_partition_info_get_rxbuf(&st, &uuid, NULL, &count), -EINVAL, NULL);
}
```

Run twister → CONFIRM fail.

- [ ] **Step 3: Implement** `ffa_rx_release` and `ffa_partition_info_get_rxbuf` in ffa_msg.c. Marshal the 4 UUID words from `uuid->bytes` via `memcpy` into four `uint32_t` (a1..a4). Hold `st->lock` across invoke→copy→release. Set the count-only flag when `out==NULL || *count==0` (and version > 1.0). Record size = a3 when version > 1.0 else `FFA_1_0_PARTITION_INFO_SZ`. Copy at most `min(reported, *count)` records; decode id/exec/props (LE) and uuid (only when record size > 8). Write reported count back into `*count`. Release RX unless count-only.

```c
int ffa_rx_release(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};

	args.a0 = FFA_RX_RELEASE;
	ffa_invoke(st, &args, &ret);
	if ((uint32_t)ret.a0 == FFA_ERROR) {
		return ffa_to_errno((int)ret.a2);
	}
	return 0;
}

int ffa_partition_info_get_rxbuf(struct ffa_drv_state *st,
				 const struct ffa_uuid *uuid,
				 struct ffa_partition_info *out, size_t *count)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs ret = {0};
	uint32_t u[4];
	uint32_t flags = 0;
	uint32_t reported, rec_sz;
	int rc = 0;
	bool count_only = (out == NULL) || (*count == 0);

	memcpy(u, uuid->bytes, sizeof(u));

	if (count_only && st->version > FFA_VERSION_1_0) {
		flags = FFA_PARTITION_INFO_GET_COUNT_ONLY;
	}

	k_mutex_lock(&st->lock, K_FOREVER);

	args.a0 = FFA_PARTITION_INFO_GET;
	args.a1 = u[0]; args.a2 = u[1]; args.a3 = u[2]; args.a4 = u[3];
	args.a5 = flags;
	ffa_invoke(st, &args, &ret);

	if ((uint32_t)ret.a0 == FFA_ERROR) {
		rc = ffa_to_errno((int)ret.a2);
		goto out_unlock;
	}

	reported = (uint32_t)ret.a2;
	rec_sz = (st->version > FFA_VERSION_1_0) ? (uint32_t)ret.a3
						 : FFA_1_0_PARTITION_INFO_SZ;

	if (out != NULL && !count_only) {
		uint32_t n = (reported < *count) ? reported : (uint32_t)*count;
		const uint8_t *base = st->rx_buf;

		for (uint32_t i = 0; i < n; i++) {
			const uint8_t *r = base + (size_t)i * rec_sz;
			uint16_t id16, exec16;
			uint32_t props32;

			memcpy(&id16, r + 0, sizeof(id16));
			memcpy(&exec16, r + 2, sizeof(exec16));
			memcpy(&props32, r + 4, sizeof(props32));
			out[i].id = id16;
			out[i].exec_ctxt = exec16;
			out[i].properties = props32;
			memset(&out[i].uuid, 0, sizeof(out[i].uuid));
			if (rec_sz > FFA_1_0_PARTITION_INFO_SZ) {
				memcpy(out[i].uuid.bytes, r + 8, sizeof(out[i].uuid.bytes));
			}
		}
	}

	if (!count_only) {
		(void)ffa_rx_release(st);
	}
	*count = reported;

out_unlock:
	k_mutex_unlock(&st->lock);
	return rc;
}
```

Run twister → CONFIRM pass (20 + 3 = 23). Commit:
```bash
git commit --no-gpg-sign -m "subsys: firmware: ffa: partition discovery (RX-buffer variant)

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 5: `_REGS` variant + public wrappers + version gating (TDD)

**Files:**
- Modify: `subsys/firmware/ffa/ffa_msg.c`, `ffa_internal.h`, `tests/.../main.c`

**Interfaces:**
- Consumes: Tasks 2-4, `ffa_query_feature`, `ffa_is_available` state.
- Produces: internal `int ffa_partition_info_get_regs(struct ffa_drv_state *st, const struct ffa_uuid *uuid, struct ffa_partition_info *out, size_t *count);` and the PUBLIC wrappers `ffa_partition_info_get`, `ffa_msg_send_direct_req`, `ffa_msg_send_direct_req2` that operate on the singleton `ffa_state` (declared `extern` from ffa_core.c or via an accessor).

- [ ] **Step 1: Expose the singleton to ffa_msg.c.** In `ffa_core.c`, change `static struct ffa_drv_state ffa_state;` to non-static and declare `extern struct ffa_drv_state ffa_state;` in `ffa_internal.h` (internal header, not public). Keep the buffers static. (Alternatively add an internal accessor `struct ffa_drv_state *ffa_get_state(void);` — pick one and be consistent; extern is simplest and matches the internal-header pattern.)

- [ ] **Step 2: Write failing tests** for the `_REGS` decode (single-shot: count in a2 low bits, one record packed at a3..a5) and for the public wrappers' unavailability behavior:

```c
ZTEST(ffa_core, test_partition_info_regs_decode)
{
	struct ffa_drv_state st = { .version = FFA_VERSION_1_2 };
	struct ffa_uuid uuid = {0};
	struct ffa_partition_info out[1];
	size_t count = 1;

	mock_script_reset();
	ffa_test_set_conduit(mock_script_conduit);
	/* a2: last_idx=0 (count=1), cur_idx=0, tag=0, size=... ; record in a3..a5 */
	struct arm_smccc_1_2_regs r = { .a0 = FFA_SUCCESS_64, .a2 = 0 /*last_idx 0 => count 1*/ };
	r.a3 = (uint64_t)0x8001 | ((uint64_t)1 << 16) | ((uint64_t)0x3 << 32); /* id, exec, props */
	r.a4 = 0x0706050403020100UL; /* uuid[0..7] */
	r.a5 = 0x0F0E0D0C0B0A0908UL; /* uuid[8..15] */
	mock_script[0] = r;
	mock_script_len = 1;

	zassert_equal(ffa_partition_info_get_regs(&st, &uuid, out, &count), 0, NULL);
	zassert_equal(mock_seen[0].a0, FFA_PARTITION_INFO_GET_REGS, NULL);
	zassert_equal(count, 1, NULL);
	zassert_equal(out[0].id, 0x8001, NULL);
	zassert_equal(out[0].exec_ctxt, 1, NULL);
	zassert_equal(out[0].properties, 0x3, NULL);
	zassert_equal(out[0].uuid.bytes[0], 0, NULL);
	zassert_equal(out[0].uuid.bytes[15], 15, NULL);
}

ZTEST(ffa_core, test_public_msg_unavailable)
{
	struct ffa_send_direct_data d = {0};
	struct ffa_uuid uuid = {0};
	struct ffa_send_direct_data2 d2 = {0};
	size_t count = 0;

	if (!ffa_is_available()) {
		zassert_equal(ffa_partition_info_get(&uuid, NULL, &count), -EAGAIN, NULL);
		zassert_equal(ffa_msg_send_direct_req(0x1, &d), -EAGAIN, NULL);
		zassert_equal(ffa_msg_send_direct_req2(0x1, &uuid, &d2), -EAGAIN, NULL);
	}
}
```

Run twister → CONFIRM fail.

- [ ] **Step 3: Implement** `ffa_partition_info_get_regs` (iterate the packed records per the Global Constraints convention; decode word0 via `FFA_PIG_REC_*`, uuid from the next two words via `memcpy`; loop with `start_idx`/`cur_idx` until `cur_idx >= count-1`). Then the internal dispatcher `ffa_partition_info_get_impl(st, ...)` that chooses `_regs` iff `st->version >= FFA_VERSION_1_2 && ffa_query_feature(st, FFA_PARTITION_INFO_GET_REGS, NULL) == 0`, else `_rxbuf`. Then the three public wrappers:

```c
int ffa_partition_info_get(const struct ffa_uuid *uuid,
			   struct ffa_partition_info *out, size_t *count)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	return ffa_partition_info_get_impl(&ffa_state, uuid, out, count);
}

int ffa_msg_send_direct_req(uint16_t dst, struct ffa_send_direct_data *data)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	/* NS endpoint talks to a 64-bit SP by default; 32-bit selection is a
	 * per-partition property discovered via partition info (caller-driven
	 * later). SP-2b uses native 64-bit. */
	return ffa_send_direct_req(&ffa_state, dst, false, data);
}

int ffa_msg_send_direct_req2(uint16_t dst, const struct ffa_uuid *uuid,
			     struct ffa_send_direct_data2 *data)
{
	if (!ffa_is_available()) {
		return -EAGAIN;
	}
	return ffa_send_direct_req2(&ffa_state, dst, uuid, data);
}
```

Run twister → CONFIRM pass (23 + 2 = 25). Commit:
```bash
git commit --no-gpg-sign -m "subsys: firmware: ffa: partition info REGS variant and public API

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 6: regression + docs update

**Files:**
- Modify: `doc/services/firmware/ffa.rst`
- Modify: current `doc/releases/release-notes-*.rst`

- [ ] **Step 1: Regression** — run the SP-1 test + the full ffa suite; confirm all pass and hello_world still builds default-off.
```bash
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west twister -p qemu_cortex_a53 -T tests/arch/arm64/smccc_1_2 -T tests/subsys/firmware/ffa'
```
- [ ] **Step 2: Docs** — extend `doc/services/firmware/ffa.rst`: document the new public API (`ffa_partition_info_get`, `ffa_msg_send_direct_req`, `ffa_msg_send_direct_req2`, `struct ffa_partition_info`, `struct ffa_send_direct_data[2]`). Since the page uses `.. doxygengroup:: ffa_core_api`, ensure the new decls are inside the group fence in the header (they are, from Task 1) — the group will render them automatically; add only prose describing discovery + messaging. Update the release note bullet to mention discovery + direct messaging.
- [ ] **Step 3: Commit**
```bash
git commit --no-gpg-sign -m "doc: firmware: ffa: document discovery and direct messaging

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

## Self-Review

**Spec coverage (SP-2b slice of the design doc):**
- "`FFA_PARTITION_INFO_GET[_REGS]` (with RX-buffer and register variants; strips the FF-A 1.0 8-byte partition record when negotiated version is 1.0)" → Tasks 4 (RX-buffer, rec_sz=8 on v1.0, no uuid) + 5 (`_REGS`, version-gated dispatch). ✅
- "`MSG_SEND_DIRECT_REQ`/`REQ2` + RESP round-trips" → Tasks 2, 3, with the `FFA_INTERRUPT`/`FFA_YIELD`→`FFA_RUN` completion loop. ✅
- "the flat public API (`ffa_partition_info_get`, `ffa_msg_send_direct_req[2]`, etc.)" → Task 5 public wrappers, added to `include/zephyr/firmware/ffa.h` inside the `ffa_core_api` group. ✅
- Reuses SP-2a `ffa_invoke`/`ffa_to_errno`/RX buffer+`st->lock`/negotiated version → yes, throughout. ✅
- Testing without a live responder → mock conduit + scripted multi-response for the completion loop; RX-buffer tests pre-populate a local buffer. ✅

**Placeholder scan:** all code blocks concrete; hex FIDs, packing, and unpacking are literal.

**Type consistency:** internal signatures (`ffa_send_direct_req[2]`, `ffa_partition_info_get_rxbuf/_regs/_impl`, `ffa_rx_release`) are declared in `ffa_internal.h` and used identically in `ffa_msg.c` and the tests. Public types (`ffa_partition_info`, `ffa_send_direct_data[2]`) are single-sourced in the public header. The `&args.a4`/`&ret.a4` array walks are guarded by a `BUILD_ASSERT` on the SMCCC struct layout.

**Endianness/alignment:** AArch64 is little-endian; UUID and record fields are read via `memcpy` (no unaligned pointer casts), so a `uint64_t` read of `bytes[0..7]` yields the LE value the ABI expects.

## Notes for later phases
- SP-3 (memory sharing) adds `ffa_mem.c` and will reuse the TX buffer + `st->lock` and the descriptor helpers.
- SP-5 (OP-TEE transport) is the first real consumer of `ffa_partition_info_get` (find OP-TEE by UUID) and `ffa_msg_send_direct_req2` (carry `OPTEE_FFA_*`); it also provides the live end-to-end exercise on OP-TEE-under-QEMU. The 32-bit-mode selection for `ffa_msg_send_direct_req` becomes caller-driven there (from discovered partition properties).
