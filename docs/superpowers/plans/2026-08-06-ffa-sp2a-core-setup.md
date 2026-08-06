# FF-A SP-2a: FF-A core setup (init / version negotiation / RXTX map) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up the reusable FF-A core subsystem skeleton in Zephyr — `CONFIG_ARM_FFA`, the public header `include/zephyr/firmware/ffa.h`, internal ABI constants, and the setup path (conduit detect, `FFA_VERSION` negotiation with graceful downgrade, `FFA_ID_GET`, `FFA_FEATURES` probe, RX/TX buffer allocation + `FFA_RXTX_MAP`, and a central `ffa_to_errno()` decoder) — all unit-testable without a live FF-A responder.

**Architecture:** New subsystem under `subsys/firmware/ffa/` layered directly on SP-1's SMCCC 1.2 primitive (`arm_smccc_1_2_smc/hvc`). A singleton driver-state struct holds the negotiated version, our VM/endpoint ID, the conduit (SMC/HVC), and the DMA-capable RX/TX buffers, guarded by a `k_mutex`. Every SPMC round-trip goes through one internal `ffa_invoke()` helper that marshals a `struct arm_smccc_1_2_regs`, so version negotiation, ID discovery, feature probing, and RXTX-map are each thin functions over it. The conduit-issuing seam (`ffa_invoke()`) is compile-time swappable for a mock so all logic is host/target unit-testable without EL3/secure world. `ffa_to_errno()` centralises the `FFA_ERROR` → `-errno` mapping. Function IDs and descriptor constants are ported from `linux/include/linux/arm_ffa.h` (canonical hex cross-checked against `optee_os/core/arch/arm/include/ffa.h`).

**Tech Stack:** Zephyr (AArch64), C, Kconfig, CMake, Ztest, `west`/twister on `qemu_cortex_a53`. Build+test executed on the fio server via `.tools/fio-push` + `.tools/fio-run` (see `WORKFLOW.md`); authoring is local.

## Global Constraints

- **Builds on SP-1 (realized):** public header `include/zephyr/arch/arm64/arm-smccc.h`, gated by `CONFIG_ARM_SMCCC_1_2`. `struct arm_smccc_1_2_regs { unsigned long a0..a17; }` — 18 contiguous `unsigned long`, `offsetof(aN)==N*8`. `void arm_smccc_1_2_smc(const struct arm_smccc_1_2_regs *args, struct arm_smccc_1_2_regs *res);` and `..._hvc(...)`. `CONFIG_ARM_FFA` **must `select ARM_SMCCC_1_2`**.
- **Target arch:** AArch64 only. `CONFIG_ARM_FFA depends on ARM64` (and thus `HAS_ARM_SMCCC`). Must not build for other arches.
- **FF-A version:** offer **1.2** (`0x00010002`), accept the SPMC's returned version, store the negotiated value, and feature-gate at runtime. Downgrade path to 1.1 (`0x00010001`) / 1.0 (`0x00010000`). If the SPMC returns `FFA_RET_NOT_SUPPORTED` (`-1`) in w0 (or `FFA_ERROR`) for `FFA_VERSION`, init fails cleanly with `-ENOTSUP`.
- **Version encoding:** bit31 must be 0 in our offered input; major = bits[30:16], minor = bits[15:0]. `FFA_VERSION_1_2 = (1<<16)|2`.
- **Canonical FF-A function IDs (SMC, fast call, owner=Standard):** 32-bit convention = `0x84000000 | fn`, 64-bit = `0xC4000000 | fn`. Exact values used in this plan: `FFA_ERROR=0x84000060`, `FFA_SUCCESS_32=0x84000061`, `FFA_SUCCESS_64=0xC4000061`, `FFA_INTERRUPT=0x84000062`, `FFA_VERSION=0x84000063`, `FFA_FEATURES=0x84000064`, `FFA_RX_RELEASE=0x84000065`, `FFA_RXTX_MAP_64=0xC4000066`, `FFA_RXTX_UNMAP=0x84000067`, `FFA_ID_GET=0x84000069`.
- **FF-A error codes** (returned in w2 of an `FFA_ERROR`, or as a negative w0): `SUCCESS=0, NOT_SUPPORTED=-1, INVALID_PARAMETERS=-2, NO_MEMORY=-3, BUSY=-4, INTERRUPTED=-5, DENIED=-6, RETRY=-7, ABORTED=-8, NO_DATA=-9`.
- **RXTX buffers:** default 4 KiB each (`FFA_PAGE_SIZE = 0x1000`), page-aligned; count in `FFA_RXTX_MAP` is in 4 KiB pages. Buffers are static, `__aligned(4096)`, in a DMA-safe/non-cached-agnostic region (plain `.bss` is acceptable for QEMU; document the assumption).
- **License header:** Apache-2.0, matching surrounding files. New public/subsys files carry `Copyright 2026 Qualcomm Innovation Center, Inc.` (match the SP-1 test file style).
- **SMC/HVC cannot execute on host/native_sim and `qemu_cortex_a53` has no FF-A responder.** All tests mock the conduit seam or test pure functions; no live SMC. A live `FFA_VERSION` round-trip belongs to SP-5 (OP-TEE-under-QEMU bring-up).
- **Deferred-from-SP-1 (fold in here):** add `BUILD_ASSERT(sizeof(struct arm_smccc_1_2_regs)==18*sizeof(unsigned long))` and `BUILD_ASSERT(offsetof(struct arm_smccc_1_2_regs, a17)==17*sizeof(unsigned long))` in the first production `.c` that includes `arm-smccc.h` (i.e. `ffa_core.c`).
- **Commits:** `git commit --no-gpg-sign` with trailer `Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>`. No `Co-Authored-By`. Stay on branch `ffa-optee-support`.
- **Scope of SP-2a:** setup only. Partition-info discovery and direct messaging are **SP-2b** — do not implement them here. Memory sharing (SP-3) and notifications (SP-4) are out of scope.

---

## File Structure

- `include/zephyr/firmware/ffa.h` (create) — public API + public data types for SP-2a: `struct ffa_uuid`, version accessors, `ffa_is_available()`, `ffa_version()`, `ffa_id_get()`, `ffa_features()`. Later slices extend this file.
- `subsys/firmware/ffa/ffa_internal.h` (create) — internal ABI constants (function IDs, `FFA_RET_*`, version macros, RXTX feature encodings) ported from the Linux header, plus the internal driver-state struct and the `ffa_invoke()`/mock seam declaration.
- `subsys/firmware/ffa/ffa_core.c` (create) — driver state singleton, `ffa_invoke()` (real conduit path), `ffa_to_errno()`, version negotiation, `FFA_ID_GET`, `FFA_FEATURES`, RXTX buffer alloc + `FFA_RXTX_MAP`, and the `SYS_INIT` entry `ffa_init()`. Holds the SP-1 `BUILD_ASSERT`s.
- `subsys/firmware/ffa/Kconfig` (create) — `config ARM_FFA` (+ `ARM_FFA_RXTX_PAGES`, `ARM_FFA_LOG_LEVEL`).
- `subsys/firmware/ffa/CMakeLists.txt` (create) — build the two `.c`/headers under `CONFIG_ARM_FFA`.
- `subsys/Kconfig` (modify) — `source "subsys/firmware/Kconfig"` in the sorted block.
- `subsys/firmware/Kconfig` (create) — `menu "Firmware"` + `source "subsys/firmware/ffa/Kconfig"` + `endmenu` (new `firmware` subsys container).
- `subsys/CMakeLists.txt` (modify) — `add_subdirectory_ifdef(CONFIG_ARM_FFA firmware)` (and a `subsys/firmware/CMakeLists.txt` that recurses into `ffa`).
- `subsys/firmware/CMakeLists.txt` (create) — `add_subdirectory_ifdef(CONFIG_ARM_FFA ffa)`.
- `tests/subsys/firmware/ffa/` (create) — Ztest suite `subsys.firmware.ffa.core`: mocks `ffa_invoke()` to drive negotiation/downgrade, `ffa_to_errno()` table, ID/feature parsing, and RXTX-map argument marshalling — all without a live responder.
  - `CMakeLists.txt`, `prj.conf`, `testcase.yaml`, `src/main.c`.

**Mock seam design (decisive detail):** `ffa_core.c` calls the conduit through a function pointer `ffa_conduit_fn` of type `void (*)(const struct arm_smccc_1_2_regs *, struct arm_smccc_1_2_regs *)`, defaulting to `arm_smccc_1_2_smc`/`_hvc` chosen at init by conduit detect. A test-only hook `ffa_test_set_conduit(fn)` (compiled only under `CONFIG_ZTEST`) lets the test install a mock. This keeps production code identical while making every code path unit-testable. All internal setup functions take an explicit `struct ffa_drv_state *` so the test can drive them on a local state object.

---

### Task 1: Kconfig + subsystem wiring + empty build

**Files:**
- Create: `subsys/firmware/Kconfig`
- Create: `subsys/firmware/ffa/Kconfig`
- Create: `subsys/firmware/CMakeLists.txt`
- Create: `subsys/firmware/ffa/CMakeLists.txt`
- Modify: `subsys/Kconfig`
- Modify: `subsys/CMakeLists.txt`

**Interfaces:**
- Consumes: `CONFIG_ARM_SMCCC_1_2` (SP-1), `ARM64`, `HAS_ARM_SMCCC`.
- Produces: `CONFIG_ARM_FFA` (bool, selects `ARM_SMCCC_1_2`), `CONFIG_ARM_FFA_RXTX_PAGES` (int, default 1), `CONFIG_ARM_FFA_LOG_LEVEL`. A buildable (empty) `subsys/firmware/ffa` compilation unit gated on `CONFIG_ARM_FFA`.

- [ ] **Step 1: Create the ffa Kconfig**

`subsys/firmware/ffa/Kconfig`:

```kconfig
# Copyright 2026 Qualcomm Innovation Center, Inc.
# SPDX-License-Identifier: Apache-2.0

config ARM_FFA
	bool "Arm Firmware Framework for A-profile (FF-A) core"
	depends on ARM64 && HAS_ARM_SMCCC
	select ARM_SMCCC_1_2
	help
	  Enable the reusable FF-A core subsystem: version negotiation,
	  endpoint-ID discovery, feature probing and RX/TX buffer mapping,
	  built on the SMCCC v1.2 extended-register call primitive. Zephyr
	  acts as the Normal-World FF-A endpoint (e.g. to talk to OP-TEE).

if ARM_FFA

config ARM_FFA_RXTX_PAGES
	int "FF-A RX/TX buffer size (in 4 KiB pages)"
	default 1
	range 1 16
	help
	  Size of each of the RX and TX buffers mapped with FFA_RXTX_MAP,
	  expressed in 4 KiB FF-A pages. The default of 1 page (4 KiB) is
	  the FF-A minimum and is sufficient for partition discovery and
	  direct messaging.

module = ARM_FFA
module-str = arm_ffa
source "subsys/logging/Kconfig.template.log_config"

endif # ARM_FFA
```

- [ ] **Step 2: Create the firmware container Kconfig**

`subsys/firmware/Kconfig`:

```kconfig
# Copyright 2026 Qualcomm Innovation Center, Inc.
# SPDX-License-Identifier: Apache-2.0

menu "Firmware"

source "subsys/firmware/ffa/Kconfig"

endmenu
```

- [ ] **Step 3: Wire into subsys/Kconfig**

In `subsys/Kconfig`, inside the `# zephyr-keep-sorted-start ... stop` block of `source` lines, add (alphabetical position — after `source "subsys/fs/Kconfig"` if present, else keep the block sorted; `firmware` sorts before `fs`):

```kconfig
source "subsys/firmware/Kconfig"
```

Verify with the surrounding lines so the keep-sorted check passes (place it before `source "subsys/fs/..."`/`source "subsys/gnss/..."` per the existing ordering).

- [ ] **Step 4: Create CMake wiring**

`subsys/firmware/CMakeLists.txt`:

```cmake
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_ARM_FFA ffa)
```

`subsys/firmware/ffa/CMakeLists.txt`:

```cmake
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(ffa_core.c)
zephyr_include_directories(.)
```

In `subsys/CMakeLists.txt`, add to the `add_subdirectory_ifdef` group (keep-sorted region uses unconditional `add_subdirectory` for always-built dirs and a separate ifdef group below; place this in the ifdef group):

```cmake
add_subdirectory_ifdef(CONFIG_ARM_FFA firmware)
```

- [ ] **Step 5: Create a minimal ffa_core.c so the library links**

`subsys/firmware/ffa/ffa_core.c` (placeholder that compiles; real content lands in Tasks 2-5):

```c
/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Real implementation is added in subsequent tasks. */
```

- [ ] **Step 6: Build with CONFIG_ARM_FFA=y**

Commit locally first, then build on fio:

```bash
# local
git add subsys/firmware subsys/Kconfig subsys/CMakeLists.txt
git commit --no-gpg-sign -m "subsys: firmware: add FF-A core Kconfig and build scaffolding

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
/home/jramirez/Work/zephyr/.tools/fio-push
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west build -p always -b qemu_cortex_a53 samples/hello_world -- -DCONFIG_ARM_FFA=y'
```
Expected: PASS (config resolves, `ARM_SMCCC_1_2` auto-selected, empty library builds). Confirm `CONFIG_ARM_SMCCC_1_2=y` appears in the generated `.config`.

- [ ] **Step 7: Commit** (already committed in Step 6; if build required a fix, amend and re-push).

---

### Task 2: Internal ABI constants header + `ffa_to_errno()` (TDD)

**Files:**
- Create: `subsys/firmware/ffa/ffa_internal.h`
- Modify: `subsys/firmware/ffa/ffa_core.c`
- Create: `tests/subsys/firmware/ffa/CMakeLists.txt`
- Create: `tests/subsys/firmware/ffa/prj.conf`
- Create: `tests/subsys/firmware/ffa/testcase.yaml`
- Create: `tests/subsys/firmware/ffa/src/main.c`

**Interfaces:**
- Consumes: nothing new (constants are self-contained).
- Produces:
  - `ffa_internal.h` with function-ID, error-code, and version macros (values per Global Constraints).
  - `int ffa_to_errno(int ffa_ret);` (in `ffa_core.c`, declared in `ffa_internal.h`) mapping `FFA_RET_*` → `-errno`: `SUCCESS→0, NOT_SUPPORTED→-ENOTSUP, INVALID_PARAMETERS→-EINVAL, NO_MEMORY→-ENOMEM, BUSY→-EBUSY, INTERRUPTED→-EINTR, DENIED→-EACCES, RETRY→-EAGAIN, ABORTED→-ECANCELED, NO_DATA→-ENODATA`, unknown→`-EINVAL`.

- [ ] **Step 1: Write ffa_internal.h**

`subsys/firmware/ffa/ffa_internal.h`:

```c
/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal FF-A ABI constants and driver state. Values ported from
 * linux/include/linux/arm_ffa.h and cross-checked against
 * optee_os/core/arch/arm/include/ffa.h.
 */

#ifndef ZEPHYR_SUBSYS_FIRMWARE_FFA_FFA_INTERNAL_H_
#define ZEPHYR_SUBSYS_FIRMWARE_FFA_FFA_INTERNAL_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arm64/arm-smccc.h>

/* FF-A function IDs (SMC, fast call, owner=Standard). */
#define FFA_ERROR                 0x84000060U
#define FFA_SUCCESS_32            0x84000061U
#define FFA_SUCCESS_64            0xC4000061U
#define FFA_INTERRUPT             0x84000062U
#define FFA_VERSION               0x84000063U
#define FFA_FEATURES              0x84000064U
#define FFA_RX_RELEASE            0x84000065U
#define FFA_RXTX_MAP_64           0xC4000066U
#define FFA_RXTX_UNMAP            0x84000067U
#define FFA_PARTITION_INFO_GET    0x84000068U
#define FFA_ID_GET                0x84000069U

/* FF-A return/error codes (in w2 of FFA_ERROR, or negative w0). */
#define FFA_RET_SUCCESS            0
#define FFA_RET_NOT_SUPPORTED      (-1)
#define FFA_RET_INVALID_PARAMETERS (-2)
#define FFA_RET_NO_MEMORY          (-3)
#define FFA_RET_BUSY               (-4)
#define FFA_RET_INTERRUPTED        (-5)
#define FFA_RET_DENIED             (-6)
#define FFA_RET_RETRY              (-7)
#define FFA_RET_ABORTED            (-8)
#define FFA_RET_NO_DATA            (-9)

/* Version encoding: bits[30:16]=major, bits[15:0]=minor; bit31 must be 0. */
#define FFA_MAJOR_SHIFT            16
#define FFA_MAJOR_MASK             0x7FFFU
#define FFA_MINOR_MASK             0xFFFFU
#define FFA_PACK_VERSION(major, minor) \
	((((uint32_t)(major) & FFA_MAJOR_MASK) << FFA_MAJOR_SHIFT) | \
	 ((uint32_t)(minor) & FFA_MINOR_MASK))
#define FFA_VERSION_MAJOR(v)       (((v) >> FFA_MAJOR_SHIFT) & FFA_MAJOR_MASK)
#define FFA_VERSION_MINOR(v)       ((v) & FFA_MINOR_MASK)
#define FFA_VERSION_1_0            FFA_PACK_VERSION(1, 0)
#define FFA_VERSION_1_1            FFA_PACK_VERSION(1, 1)
#define FFA_VERSION_1_2            FFA_PACK_VERSION(1, 2)

/* FFA_VERSION returns a negative 32-bit value (bit31 set) on NOT_SUPPORTED. */
#define FFA_VERSION_NOT_SUPPORTED  0xFFFFFFFFU

/* FF-A 4 KiB page (spec constant, not the kernel translation granule). */
#define FFA_PAGE_SIZE              0x1000U

/* FFA_FEATURES RXTX minimum-buffer-size encodings (w2 bits[1:0]). */
#define FFA_FEAT_RXTX_MIN_SZ_MASK  0x3U
#define FFA_FEAT_RXTX_MIN_SZ_4K    0
#define FFA_FEAT_RXTX_MIN_SZ_64K   1
#define FFA_FEAT_RXTX_MIN_SZ_16K   2

/* Internal singleton driver state. */
struct ffa_drv_state {
	bool available;             /* FF-A init succeeded */
	uint32_t version;           /* negotiated framework version */
	uint16_t vm_id;             /* our endpoint ID (FFA_ID_GET) */
	enum arm_smccc_conduit conduit;
	void *tx_buf;
	void *rx_buf;
	uint32_t rxtx_pages;        /* pages per buffer */
	struct k_mutex lock;        /* serializes RX buffer use */
};

/* Conduit seam: production uses arm_smccc_1_2_smc/hvc; tests install a mock. */
typedef void (*ffa_conduit_fn_t)(const struct arm_smccc_1_2_regs *args,
				 struct arm_smccc_1_2_regs *res);

int ffa_to_errno(int ffa_ret);

#endif /* ZEPHYR_SUBSYS_FIRMWARE_FFA_FFA_INTERNAL_H_ */
```

- [ ] **Step 2: Write the failing test for ffa_to_errno**

Replace `tests/subsys/firmware/ffa/src/main.c` with (this file grows across tasks):

```c
/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include "ffa_internal.h"

ZTEST(ffa_core, test_to_errno_mapping)
{
	zassert_equal(ffa_to_errno(FFA_RET_SUCCESS), 0, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_NOT_SUPPORTED), -ENOTSUP, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_INVALID_PARAMETERS), -EINVAL, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_NO_MEMORY), -ENOMEM, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_BUSY), -EBUSY, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_INTERRUPTED), -EINTR, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_DENIED), -EACCES, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_RETRY), -EAGAIN, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_ABORTED), -ECANCELED, NULL);
	zassert_equal(ffa_to_errno(FFA_RET_NO_DATA), -ENODATA, NULL);
	zassert_equal(ffa_to_errno(-12345), -EINVAL, "unknown maps to -EINVAL");
}

ZTEST_SUITE(ffa_core, NULL, NULL, NULL, NULL, NULL);
```

`tests/subsys/firmware/ffa/prj.conf`:

```conf
CONFIG_ZTEST=y
CONFIG_ARM_FFA=y
```

`tests/subsys/firmware/ffa/CMakeLists.txt`:

```cmake
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ffa_core)

target_sources(app PRIVATE src/main.c)
# Reach the subsystem's internal header for white-box testing.
target_include_directories(app PRIVATE
  ${ZEPHYR_BASE}/subsys/firmware/ffa)
```

`tests/subsys/firmware/ffa/testcase.yaml`:

```yaml
common:
  tags: arm64 ffa firmware
tests:
  subsys.firmware.ffa.core:
    platform_allow:
      - qemu_cortex_a53
    integration_platforms:
      - qemu_cortex_a53
```

- [ ] **Step 3: Run the test to see it fail (ffa_to_errno undefined)**

```bash
/home/jramirez/Work/zephyr/.tools/fio-push
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west twister -p qemu_cortex_a53 -T tests/subsys/firmware/ffa'
```
Expected: FAIL — link error, `ffa_to_errno` undefined. (Commit only after it passes; push is fine for building.)

- [ ] **Step 4: Implement ffa_to_errno in ffa_core.c**

Replace the placeholder body of `subsys/firmware/ffa/ffa_core.c` with:

```c
/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include "ffa_internal.h"

/* SP-1 deferred layout guards: the asm in smccc-call.S relies on these. */
BUILD_ASSERT(sizeof(struct arm_smccc_1_2_regs) == 18 * sizeof(unsigned long),
	     "arm_smccc_1_2_regs must be 18 contiguous longs");
BUILD_ASSERT(offsetof(struct arm_smccc_1_2_regs, a17) == 17 * sizeof(unsigned long),
	     "arm_smccc_1_2_regs a17 must be at offset 17*8");

int ffa_to_errno(int ffa_ret)
{
	switch (ffa_ret) {
	case FFA_RET_SUCCESS:            return 0;
	case FFA_RET_NOT_SUPPORTED:      return -ENOTSUP;
	case FFA_RET_INVALID_PARAMETERS: return -EINVAL;
	case FFA_RET_NO_MEMORY:          return -ENOMEM;
	case FFA_RET_BUSY:               return -EBUSY;
	case FFA_RET_INTERRUPTED:        return -EINTR;
	case FFA_RET_DENIED:             return -EACCES;
	case FFA_RET_RETRY:              return -EAGAIN;
	case FFA_RET_ABORTED:            return -ECANCELED;
	case FFA_RET_NO_DATA:            return -ENODATA;
	default:                         return -EINVAL;
	}
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
/home/jramirez/Work/zephyr/.tools/fio-push
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west twister -p qemu_cortex_a53 -T tests/subsys/firmware/ffa'
```
Expected: PASS — `test_to_errno_mapping` passes. Also confirms the SP-1 `BUILD_ASSERT`s hold.

- [ ] **Step 6: Commit**

```bash
git add subsys/firmware/ffa/ffa_internal.h subsys/firmware/ffa/ffa_core.c tests/subsys/firmware/ffa
git commit --no-gpg-sign -m "subsys: firmware: ffa: add ABI constants and error decode

Adds internal FF-A ABI constants (function IDs, return codes, version
encoding) ported from the Linux arm_ffa.h, the central ffa_to_errno()
decoder, and the SMCCC 1.2 struct-layout BUILD_ASSERTs deferred from SP-1.

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 3: Conduit seam + `ffa_invoke()` + version negotiation (TDD, mocked conduit)

**Files:**
- Modify: `subsys/firmware/ffa/ffa_internal.h`
- Modify: `subsys/firmware/ffa/ffa_core.c`
- Modify: `tests/subsys/firmware/ffa/src/main.c`

**Interfaces:**
- Consumes: `struct arm_smccc_1_2_regs`, `arm_smccc_1_2_smc/hvc` (SP-1); `ffa_to_errno` (Task 2).
- Produces (internal, declared in `ffa_internal.h`):
  - `void ffa_invoke(struct ffa_drv_state *st, struct arm_smccc_1_2_regs *args, struct arm_smccc_1_2_regs *res);`
  - `int ffa_negotiate_version(struct ffa_drv_state *st);` — puts `FFA_VERSION` with offered `FFA_VERSION_1_2` in a1, reads returned version from w0/a0, applies downgrade rules, stores into `st->version`; returns 0 or `-ENOTSUP`.
  - Test hook (only under `CONFIG_ZTEST`): `void ffa_test_set_conduit(ffa_conduit_fn_t fn);`
- Negotiation rules (exact): offered = `FFA_VERSION_1_2`. Interpret a0 as `uint32_t ret`. If `ret == FFA_VERSION_NOT_SUPPORTED` (0xFFFFFFFF) or bit31 set → `-ENOTSUP`. Else `major = FFA_VERSION_MAJOR(ret)`, `minor = FFA_VERSION_MINOR(ret)`. If `major != 1` → `-ENOTSUP`. Store `st->version = min(offered, ret)` clamped to the set {1.0,1.1,1.2}: if `minor >= 2` store `FFA_VERSION_1_2`; `minor == 1` → `1_1`; `minor == 0` → `1_0`. Return 0.

- [ ] **Step 1: Add declarations to ffa_internal.h**

Append inside the header (before the closing `#endif`):

```c
void ffa_invoke(struct ffa_drv_state *st, struct arm_smccc_1_2_regs *args,
		struct arm_smccc_1_2_regs *res);
int ffa_negotiate_version(struct ffa_drv_state *st);

#ifdef CONFIG_ZTEST
void ffa_test_set_conduit(ffa_conduit_fn_t fn);
#endif
```

- [ ] **Step 2: Write the failing negotiation tests**

Add to `tests/subsys/firmware/ffa/src/main.c` (before the `ZTEST_SUITE` line):

```c
#include <string.h>
#include <zephyr/arch/arm64/arm-smccc.h>

/* Mock conduit: canned response + capture of the last request. */
static struct arm_smccc_1_2_regs mock_last_args;
static struct arm_smccc_1_2_regs mock_next_res;

static void mock_conduit(const struct arm_smccc_1_2_regs *args,
			 struct arm_smccc_1_2_regs *res)
{
	mock_last_args = *args;
	*res = mock_next_res;
}

static void set_mock_res_a0(unsigned long a0)
{
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = a0;
}

ZTEST(ffa_core, test_version_offer_and_accept_1_2)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_1_2);

	zassert_equal(ffa_negotiate_version(&st), 0, NULL);
	zassert_equal(st.version, FFA_VERSION_1_2, NULL);
	/* We offered FFA_VERSION with 1.2 in a1. */
	zassert_equal(mock_last_args.a0, FFA_VERSION, "function id");
	zassert_equal(mock_last_args.a1, FFA_VERSION_1_2, "offered version");
}

ZTEST(ffa_core, test_version_downgrade_to_1_1)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_1_1);

	zassert_equal(ffa_negotiate_version(&st), 0, NULL);
	zassert_equal(st.version, FFA_VERSION_1_1, NULL);
}

ZTEST(ffa_core, test_version_downgrade_to_1_0)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_1_0);

	zassert_equal(ffa_negotiate_version(&st), 0, NULL);
	zassert_equal(st.version, FFA_VERSION_1_0, NULL);
}

ZTEST(ffa_core, test_version_not_supported)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_VERSION_NOT_SUPPORTED);

	zassert_equal(ffa_negotiate_version(&st), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_version_bad_major)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_res_a0(FFA_PACK_VERSION(2, 0));

	zassert_equal(ffa_negotiate_version(&st), -ENOTSUP, NULL);
}
```

- [ ] **Step 3: Run to see the new tests fail (undefined refs)**

```bash
/home/jramirez/Work/zephyr/.tools/fio-push
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west twister -p qemu_cortex_a53 -T tests/subsys/firmware/ffa'
```
Expected: FAIL — `ffa_test_set_conduit` / `ffa_negotiate_version` undefined.

- [ ] **Step 4: Implement the seam and negotiation in ffa_core.c**

Add to `ffa_core.c` (after `ffa_to_errno`):

```c
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(arm_ffa, CONFIG_ARM_FFA_LOG_LEVEL);

/* Default conduit set at init; overridable by tests. */
static ffa_conduit_fn_t ffa_conduit_fn;

void ffa_invoke(struct ffa_drv_state *st, struct arm_smccc_1_2_regs *args,
		struct arm_smccc_1_2_regs *res)
{
	ARG_UNUSED(st);
	__ASSERT_NO_MSG(ffa_conduit_fn != NULL);
	ffa_conduit_fn(args, res);
}

#ifdef CONFIG_ZTEST
void ffa_test_set_conduit(ffa_conduit_fn_t fn)
{
	ffa_conduit_fn = fn;
}
#endif

int ffa_negotiate_version(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};
	uint32_t ret;
	uint16_t major, minor;

	args.a0 = FFA_VERSION;
	args.a1 = FFA_VERSION_1_2;
	ffa_invoke(st, &args, &res);

	ret = (uint32_t)res.a0;
	if (ret == FFA_VERSION_NOT_SUPPORTED || (ret & 0x80000000U)) {
		LOG_ERR("FFA_VERSION not supported by SPMC");
		return -ENOTSUP;
	}

	major = FFA_VERSION_MAJOR(ret);
	minor = FFA_VERSION_MINOR(ret);
	if (major != 1) {
		LOG_ERR("Unsupported FF-A major version %u", major);
		return -ENOTSUP;
	}

	if (minor >= 2) {
		st->version = FFA_VERSION_1_2;
	} else if (minor == 1) {
		st->version = FFA_VERSION_1_1;
	} else {
		st->version = FFA_VERSION_1_0;
	}

	LOG_INF("FF-A version negotiated: 1.%u", FFA_VERSION_MINOR(st->version));
	return 0;
}
```

- [ ] **Step 5: Run to verify all negotiation tests pass**

```bash
/home/jramirez/Work/zephyr/.tools/fio-push
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west twister -p qemu_cortex_a53 -T tests/subsys/firmware/ffa'
```
Expected: PASS — all 6 `ffa_core` tests pass.

- [ ] **Step 6: Commit**

```bash
git add subsys/firmware/ffa/ffa_internal.h subsys/firmware/ffa/ffa_core.c tests/subsys/firmware/ffa/src/main.c
git commit --no-gpg-sign -m "subsys: firmware: ffa: conduit seam and version negotiation

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 4: `FFA_ID_GET` + `FFA_FEATURES` probe (TDD, mocked conduit)

**Files:**
- Modify: `subsys/firmware/ffa/ffa_internal.h`
- Modify: `subsys/firmware/ffa/ffa_core.c`
- Modify: `tests/subsys/firmware/ffa/src/main.c`

**Interfaces:**
- Consumes: `ffa_invoke`, `ffa_to_errno`, the FID macros (Tasks 2-3).
- Produces (internal, in `ffa_internal.h`):
  - `int ffa_get_id(struct ffa_drv_state *st);` — issues `FFA_ID_GET`; on `FFA_SUCCESS_32` in a0, stores `(uint16_t)(res.a2 & 0xFFFF)` into `st->vm_id`, returns 0; on `FFA_ERROR` returns `ffa_to_errno((int)res.a2)`.
  - `int ffa_query_feature(struct ffa_drv_state *st, uint32_t ffa_func_id, uint32_t *out);` — issues `FFA_FEATURES` with a1=`ffa_func_id`; on `FFA_SUCCESS_32` sets `*out = (uint32_t)res.a2` and returns 0; on `FFA_ERROR` returns the decoded errno (used to detect NOT_SUPPORTED features).
- Convention: `FFA_SUCCESS_32`/`FFA_SUCCESS_64` in a0 means success; `FFA_ERROR` in a0 means the error code is in a2 (as an `FFA_RET_*` value).

- [ ] **Step 1: Add declarations to ffa_internal.h**

```c
int ffa_get_id(struct ffa_drv_state *st);
int ffa_query_feature(struct ffa_drv_state *st, uint32_t ffa_func_id,
		      uint32_t *out);
```

- [ ] **Step 2: Write failing tests**

Add to `src/main.c`:

```c
/* Helper: canned FFA_SUCCESS_32 with a chosen a2. */
static void set_mock_success(unsigned long a2)
{
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_SUCCESS_32;
	mock_next_res.a2 = a2;
}

/* Helper: canned FFA_ERROR with a chosen error code in a2. */
static void set_mock_error(long code)
{
	memset(&mock_next_res, 0, sizeof(mock_next_res));
	mock_next_res.a0 = FFA_ERROR;
	mock_next_res.a2 = (unsigned long)code;
}

ZTEST(ffa_core, test_id_get_success)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_success(0x8001);

	zassert_equal(ffa_get_id(&st), 0, NULL);
	zassert_equal(st.vm_id, 0x8001, NULL);
	zassert_equal(mock_last_args.a0, FFA_ID_GET, "function id");
}

ZTEST(ffa_core, test_id_get_error)
{
	struct ffa_drv_state st = {0};

	ffa_test_set_conduit(mock_conduit);
	set_mock_error(FFA_RET_NOT_SUPPORTED);

	zassert_equal(ffa_get_id(&st), -ENOTSUP, NULL);
}

ZTEST(ffa_core, test_features_supported)
{
	struct ffa_drv_state st = {0};
	uint32_t props = 0xdead;

	ffa_test_set_conduit(mock_conduit);
	set_mock_success(FFA_FEAT_RXTX_MIN_SZ_4K);

	zassert_equal(ffa_query_feature(&st, FFA_RXTX_MAP_64, &props), 0, NULL);
	zassert_equal(props, FFA_FEAT_RXTX_MIN_SZ_4K, NULL);
	zassert_equal(mock_last_args.a0, FFA_FEATURES, NULL);
	zassert_equal(mock_last_args.a1, FFA_RXTX_MAP_64, NULL);
}

ZTEST(ffa_core, test_features_not_supported)
{
	struct ffa_drv_state st = {0};
	uint32_t props = 0;

	ffa_test_set_conduit(mock_conduit);
	set_mock_error(FFA_RET_NOT_SUPPORTED);

	zassert_equal(ffa_query_feature(&st, FFA_RXTX_MAP_64, &props), -ENOTSUP, NULL);
}
```

- [ ] **Step 3: Run to see failures.** (Same twister command as before.) Expected: FAIL (undefined refs).

- [ ] **Step 4: Implement in ffa_core.c**

```c
int ffa_get_id(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};

	args.a0 = FFA_ID_GET;
	ffa_invoke(st, &args, &res);

	if ((uint32_t)res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}
	st->vm_id = (uint16_t)(res.a2 & 0xFFFFU);
	return 0;
}

int ffa_query_feature(struct ffa_drv_state *st, uint32_t ffa_func_id,
		      uint32_t *out)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};

	args.a0 = FFA_FEATURES;
	args.a1 = ffa_func_id;
	ffa_invoke(st, &args, &res);

	if ((uint32_t)res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}
	if (out != NULL) {
		*out = (uint32_t)res.a2;
	}
	return 0;
}
```

- [ ] **Step 5: Run to verify pass.** Expected: PASS (all ffa_core tests).

- [ ] **Step 6: Commit**

```bash
git add subsys/firmware/ffa/ffa_internal.h subsys/firmware/ffa/ffa_core.c tests/subsys/firmware/ffa/src/main.c
git commit --no-gpg-sign -m "subsys: firmware: ffa: endpoint-id discovery and feature probe

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 5: RX/TX buffer alloc + `FFA_RXTX_MAP`, conduit detect, public header + `SYS_INIT`

**Files:**
- Modify: `subsys/firmware/ffa/ffa_internal.h`
- Modify: `subsys/firmware/ffa/ffa_core.c`
- Create: `include/zephyr/firmware/ffa.h`
- Modify: `tests/subsys/firmware/ffa/src/main.c`

**Interfaces:**
- Consumes: everything from Tasks 2-4.
- Produces:
  - Internal: `int ffa_rxtx_map(struct ffa_drv_state *st);` — issues `FFA_RXTX_MAP_64` with a1=`(uintptr_t)st->tx_buf`, a2=`(uintptr_t)st->rx_buf`, a3=`st->rxtx_pages`; on `FFA_ERROR` returns decoded errno; else 0. Buffers are module-static, `__aligned(FFA_PAGE_SIZE)`, sized `CONFIG_ARM_FFA_RXTX_PAGES * FFA_PAGE_SIZE`.
  - Internal: `enum arm_smccc_conduit ffa_detect_conduit(void);` — for SP-2a returns `SMCCC_CONDUIT_SMC` (single-Zephyr NS endpoint on QEMU uses SMC; DT-driven hvc selection is deferred to the OP-TEE transport in SP-5, matching the spec's "conduit from devicetree, as the existing OP-TEE driver does"). Document this.
  - Public (`include/zephyr/firmware/ffa.h`):
    - `struct ffa_uuid { uint8_t bytes[16]; };`
    - `bool ffa_is_available(void);`
    - `int ffa_version(uint32_t *out);` — returns `-EAGAIN` if not available, else 0 and writes negotiated version.
    - `int ffa_id_get(uint16_t *vm_id);` — returns `-EAGAIN` if not available, else 0 and writes stored `vm_id`.
  - `SYS_INIT(ffa_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT)` running: set conduit → `arm_smccc_1_2_smc`/`_hvc`; `ffa_negotiate_version`; `ffa_get_id`; `ffa_query_feature(FFA_RXTX_MAP_64)` (log if unsupported, non-fatal); `ffa_rxtx_map`; set `available=true`; `k_mutex_init`. Any hard failure logs and leaves `available=false`, returning 0 (never blocks boot).

- [ ] **Step 1: Create the public header**

`include/zephyr/firmware/ffa.h`:

```c
/*
 * Copyright 2026 Qualcomm Innovation Center, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_FIRMWARE_FFA_H_
#define ZEPHYR_INCLUDE_FIRMWARE_FFA_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A 16-byte FF-A partition UUID, little-endian byte order per the FF-A spec. */
struct ffa_uuid {
	uint8_t bytes[16];
};

/** @return true once the FF-A core has initialised and negotiated a version. */
bool ffa_is_available(void);

/**
 * @brief Get the negotiated FF-A framework version.
 * @param out receives the packed version (major<<16 | minor).
 * @retval 0 on success, -EAGAIN if FF-A is not available.
 */
int ffa_version(uint32_t *out);

/**
 * @brief Get this endpoint's FF-A ID.
 * @param vm_id receives the endpoint ID.
 * @retval 0 on success, -EAGAIN if FF-A is not available.
 */
int ffa_id_get(uint16_t *vm_id);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_FIRMWARE_FFA_H_ */
```

- [ ] **Step 2: Add internal declarations**

In `ffa_internal.h`:

```c
int ffa_rxtx_map(struct ffa_drv_state *st);
enum arm_smccc_conduit ffa_detect_conduit(void);
```

- [ ] **Step 3: Write failing tests (rxtx map marshalling + public accessors)**

Add to `src/main.c`:

```c
#include <zephyr/firmware/ffa.h>

ZTEST(ffa_core, test_rxtx_map_marshalling)
{
	static uint8_t txb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	static uint8_t rxb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	struct ffa_drv_state st = {0};

	st.tx_buf = txb;
	st.rx_buf = rxb;
	st.rxtx_pages = 1;

	ffa_test_set_conduit(mock_conduit);
	set_mock_success(0);

	zassert_equal(ffa_rxtx_map(&st), 0, NULL);
	zassert_equal(mock_last_args.a0, FFA_RXTX_MAP_64, NULL);
	zassert_equal(mock_last_args.a1, (unsigned long)(uintptr_t)txb, "tx addr");
	zassert_equal(mock_last_args.a2, (unsigned long)(uintptr_t)rxb, "rx addr");
	zassert_equal(mock_last_args.a3, 1, "page count");
}

ZTEST(ffa_core, test_rxtx_map_error)
{
	static uint8_t txb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	static uint8_t rxb[FFA_PAGE_SIZE] __aligned(FFA_PAGE_SIZE);
	struct ffa_drv_state st = { .tx_buf = txb, .rx_buf = rxb, .rxtx_pages = 1 };

	ffa_test_set_conduit(mock_conduit);
	set_mock_error(FFA_RET_NO_MEMORY);

	zassert_equal(ffa_rxtx_map(&st), -ENOMEM, NULL);
}

/* After boot SYS_INIT ran with the real (SMC) conduit but no responder, so
 * FF-A must report unavailable rather than crashing. */
ZTEST(ffa_core, test_public_accessors_when_unavailable)
{
	uint32_t v = 0;
	uint16_t id = 0;

	if (!ffa_is_available()) {
		zassert_equal(ffa_version(&v), -EAGAIN, NULL);
		zassert_equal(ffa_id_get(&id), -EAGAIN, NULL);
	} else {
		zassert_equal(ffa_version(&v), 0, NULL);
		zassert_equal(ffa_id_get(&id), 0, NULL);
	}
}
```

- [ ] **Step 4: Run to see failures.** Expected: FAIL (undefined `ffa_rxtx_map`, and public accessors unlinked).

- [ ] **Step 5: Implement in ffa_core.c**

```c
#include <zephyr/init.h>
#include <zephyr/firmware/ffa.h>

#define FFA_RXTX_BUF_SIZE (CONFIG_ARM_FFA_RXTX_PAGES * FFA_PAGE_SIZE)

static uint8_t ffa_tx_buf[FFA_RXTX_BUF_SIZE] __aligned(FFA_PAGE_SIZE);
static uint8_t ffa_rx_buf[FFA_RXTX_BUF_SIZE] __aligned(FFA_PAGE_SIZE);

static struct ffa_drv_state ffa_state;

enum arm_smccc_conduit ffa_detect_conduit(void)
{
	/*
	 * SP-2a: Zephyr as the single NS endpoint on qemu_cortex_a53 uses SMC.
	 * DT-driven conduit selection (smc/hvc) arrives with the OP-TEE FF-A
	 * transport in SP-5, mirroring the existing OP-TEE SMC driver.
	 */
	return SMCCC_CONDUIT_SMC;
}

int ffa_rxtx_map(struct ffa_drv_state *st)
{
	struct arm_smccc_1_2_regs args = {0};
	struct arm_smccc_1_2_regs res = {0};

	args.a0 = FFA_RXTX_MAP_64;
	args.a1 = (unsigned long)(uintptr_t)st->tx_buf;
	args.a2 = (unsigned long)(uintptr_t)st->rx_buf;
	args.a3 = st->rxtx_pages;
	ffa_invoke(st, &args, &res);

	if ((uint32_t)res.a0 == FFA_ERROR) {
		return ffa_to_errno((int)res.a2);
	}
	return 0;
}

bool ffa_is_available(void)
{
	return ffa_state.available;
}

int ffa_version(uint32_t *out)
{
	if (!ffa_state.available) {
		return -EAGAIN;
	}
	if (out != NULL) {
		*out = ffa_state.version;
	}
	return 0;
}

int ffa_id_get(uint16_t *vm_id)
{
	if (!ffa_state.available) {
		return -EAGAIN;
	}
	if (vm_id != NULL) {
		*vm_id = ffa_state.vm_id;
	}
	return 0;
}

static int ffa_init(void)
{
	struct ffa_drv_state *st = &ffa_state;
	int ret;

	k_mutex_init(&st->lock);
	st->tx_buf = ffa_tx_buf;
	st->rx_buf = ffa_rx_buf;
	st->rxtx_pages = CONFIG_ARM_FFA_RXTX_PAGES;
	st->conduit = ffa_detect_conduit();

#ifndef CONFIG_ZTEST
	ffa_conduit_fn = (st->conduit == SMCCC_CONDUIT_HVC)
		       ? arm_smccc_1_2_hvc : arm_smccc_1_2_smc;
#endif

	ret = ffa_negotiate_version(st);
	if (ret != 0) {
		LOG_WRN("FF-A not available: version negotiation failed (%d)", ret);
		return 0;
	}

	ret = ffa_get_id(st);
	if (ret != 0) {
		LOG_WRN("FF-A not available: ID_GET failed (%d)", ret);
		return 0;
	}

	ret = ffa_query_feature(st, FFA_RXTX_MAP_64, NULL);
	if (ret != 0) {
		LOG_INF("FFA_RXTX_MAP feature not advertised (%d)", ret);
	}

	ret = ffa_rxtx_map(st);
	if (ret != 0) {
		LOG_WRN("FF-A not available: RXTX_MAP failed (%d)", ret);
		return 0;
	}

	st->available = true;
	LOG_INF("FF-A core ready (v1.%u, id 0x%04x)",
		FFA_VERSION_MINOR(st->version), st->vm_id);
	return 0;
}

SYS_INIT(ffa_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
```

Note: under `CONFIG_ZTEST` the conduit is left for the test to install; `ffa_init` still runs but negotiation will call a NULL conduit — guard it. Adjust `ffa_init` to skip the SPMC calls when no conduit is set:

```c
	if (ffa_conduit_fn == NULL) {
		LOG_INF("FF-A: no conduit configured; core idle");
		return 0;
	}
```
Place this check immediately after the `#ifndef CONFIG_ZTEST ... #endif` conduit assignment and before `ffa_negotiate_version`. This makes boot under the test image safe (the test installs its own mock per-case on local state objects, not the singleton).

- [ ] **Step 6: Run to verify pass.** Expected: PASS — all ffa_core tests, including the marshalling and the availability accessor cases. On this board with no responder, `ffa_is_available()` is false and the accessor test takes the `-EAGAIN` branch.

- [ ] **Step 7: Commit**

```bash
git add include/zephyr/firmware/ffa.h subsys/firmware/ffa/ffa_internal.h subsys/firmware/ffa/ffa_core.c tests/subsys/firmware/ffa/src/main.c
git commit --no-gpg-sign -m "subsys: firmware: ffa: RXTX map, conduit detect, init and public API

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

### Task 6: Regression + docs + release note

**Files:**
- Modify: `doc/releases/release-notes-<next>.rst` (find the current dev release-notes file)
- Create: `doc/services/firmware/ffa.rst` (subsystem doc page) — or the nearest existing services doc tree
- Modify: `doc/services/index.rst` (or the appropriate toctree) to include the new page

**Interfaces:** none (docs + regression only).

- [ ] **Step 1: Regression — the existing SMCCC 1.2 test still passes with FF-A present**

```bash
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west twister -p qemu_cortex_a53 -T tests/arch/arm64/smccc_1_2 -T tests/subsys/firmware/ffa'
```
Expected: PASS both suites.

- [ ] **Step 2: Sanity — hello_world still builds without FF-A (default off)**

```bash
/home/jramirez/Work/zephyr/.tools/fio-run 'V=/home/jramirez/zephyrproject/.venv; export PATH="$V/bin:$PATH"; export ZEPHYR_TOOLCHAIN_VARIANT=zephyr; cd /home/jramirez/Work/zephyr/zephyr; $V/bin/west build -p always -b qemu_cortex_a53 samples/hello_world'
```
Expected: PASS; `CONFIG_ARM_FFA` absent from `.config` (default n).

- [ ] **Step 3: Write the subsystem doc page**

Create `doc/services/firmware/ffa.rst` with an overview (FF-A core, NS endpoint, version negotiation, RXTX map, `CONFIG_ARM_FFA`), the public API (`ffa_is_available`, `ffa_version`, `ffa_id_get`), and a note that discovery/messaging (SP-2b), memory sharing (SP-3) and notifications (SP-4) extend it. Add it to the services toctree. (Locate the exact toctree with `grep -rn "toctree" doc/services/index.rst` before editing — match the existing structure.)

- [ ] **Step 4: Add a release note**

In the current dev `doc/releases/release-notes-*.rst`, under the "New APIs" / "Libraries / Subsystems" section, add:

```rst
* Added the Arm Firmware Framework for A-profile (FF-A) core subsystem
  (:kconfig:option:`CONFIG_ARM_FFA`), providing version negotiation, endpoint
  ID discovery, feature probing and RX/TX buffer mapping for a Zephyr
  Normal-World endpoint. Built on the new SMCCC v1.2 call layer.
```

- [ ] **Step 5: Build the docs check is out of scope for QEMU; verify RST is well-formed by grep and commit.**

```bash
git add doc/
git commit --no-gpg-sign -m "doc: firmware: document the FF-A core subsystem

Signed-off-by: Jorge Ramirez-Ortiz <jorge.ramirez@oss.qualcomm.com>"
```

---

## Self-Review

**Spec coverage (SP-2a slice of the design doc, "SP-2 — FF-A core: setup" portion):**
- "new subsystem `subsys/firmware/ffa/` + public header `include/zephyr/firmware/ffa.h` + `CONFIG_ARM_FFA` (selects `ARM_SMCCC_1_2`)" → Tasks 1, 5. ✅
- "Conduit detect (smc/hvc)" → `ffa_detect_conduit` Task 5 (SMC now; DT hvc deferred to SP-5 per spec's own statement that conduit comes from DT "as the existing OP-TEE driver does" — documented). ✅
- "`FFA_VERSION` negotiation (offer 1.2, store negotiated, downgrade)" → Task 3. ✅
- "`FFA_ID_GET`, `FFA_FEATURES` probe" → Task 4. ✅
- "RX/TX buffer alloc + `FFA_RXTX_MAP`" → Task 5. ✅
- "central `ffa_to_errno()` decode" → Task 2. ✅
- "Port constants from `linux/include/linux/arm_ffa.h`" → Task 2 (`ffa_internal.h`), values cross-checked vs optee_os. ✅
- Testing caveat "unit-testable WITHOUT a live responder (mock the conduit or test pure functions)" → mock seam + pure-function tests throughout; no live SMC. ✅
- Deferred SP-1 `BUILD_ASSERT` folded into first production TU (`ffa_core.c`) → Task 2. ✅
- SP-2b scope (partition_info, direct messaging) explicitly excluded. ✅

**Placeholder scan:** All code blocks are complete and concrete; hex FIDs and errno mappings are literal. The only "locate the exact file" instructions are in Task 6 (docs), which are inherently tree-version-dependent and bounded by a grep step — acceptable. ✅

**Type consistency:** `struct ffa_drv_state`, `ffa_conduit_fn_t`, and every internal function signature (`ffa_invoke`, `ffa_negotiate_version`, `ffa_get_id`, `ffa_query_feature`, `ffa_rxtx_map`, `ffa_detect_conduit`) are declared in `ffa_internal.h` and used identically in `ffa_core.c` and the tests. Public API in `ffa.h` (`ffa_is_available/version/id_get`, `struct ffa_uuid`) matches the spec's flat API and the SP-2b extension plan. FID/errno constants are single-sourced in `ffa_internal.h`. ✅

**Build-order safety:** Task 1 leaves a compiling empty library; each later task is red→green→commit. The `CONFIG_ZTEST` conduit guard in `ffa_init` (Task 5) ensures the test image boots without invoking a NULL conduit. ✅

## Notes for later phases (not part of SP-2a)

- **SP-2b** extends `include/zephyr/firmware/ffa.h` with `struct ffa_partition_info`, `struct ffa_send_direct_data[2]`, `ffa_partition_info_get`, `ffa_msg_send_direct_req[2]`, adding `ffa_msg.c`. It reuses `ffa_invoke`, `ffa_to_errno`, the RX buffer + `st->lock`, and the negotiated version for the `PARTITION_INFO_GET_REGS`→RX-buffer and `DIRECT_REQ2`→`DIRECT_REQ` fallbacks.
- **SP-5** replaces `ffa_detect_conduit`'s hardcoded SMC with DT-driven selection when the OP-TEE FF-A transport lands, and provides the live `FFA_VERSION` end-to-end exercise on OP-TEE-under-QEMU.
