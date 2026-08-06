.. _ffa_core:

Arm FF-A Core Subsystem
#######################

Overview
********

The Arm Firmware Framework for A-profile (FF-A) subsystem enables a Zephyr
Normal-World endpoint to communicate with Secure-World partitions (Secure
Partitions, SPs) managed by a Secure Partition Manager Core (SPMC) such as
OP-TEE or TF-A.

Enable the subsystem with :kconfig:option:`CONFIG_ARM_FFA`. This selects the
SMCCC v1.2 call layer (:kconfig:option:`CONFIG_ARM_SMCCC_1_2`) and is
restricted to AArch64 targets.

The core setup phase (this subsystem) covers:

* **Version negotiation** — Zephyr offers FF-A 1.2 and downgrades to the
  version reported by the SPMC (1.1 or 1.0 are accepted). Initialisation fails
  cleanly with ``-ENOTSUP`` if the SPMC does not support FF-A at all.
* **Endpoint-ID discovery** — the SPMC-assigned FF-A ID for this endpoint is
  retrieved via ``FFA_ID_GET`` and cached for the lifetime of the system.
* **Feature probing** — the presence of optional FF-A features is queried
  during initialisation via ``FFA_FEATURES``.
* **RX/TX buffer mapping** — 4 KiB RX and TX buffers are registered with the
  SPMC via ``FFA_RXTX_MAP``, enabling descriptor-based communication in later
  phases.

.. note::

   RX/TX buffers are statically allocated, page-aligned ``__aligned(4096)``
   arrays in ``.bss``. This is sufficient for QEMU environments; platforms with
   cache-coherency requirements may need additional configuration.

Partition Discovery
*******************

The :c:func:`ffa_partition_info_get` function discovers Secure Partitions
registered with the SPMC.

* Pass a nil UUID (all-zero bytes) to enumerate every partition visible to
  this endpoint; pass a specific UUID to filter by service identity.
* Pass ``out = NULL`` to perform a count-only query: the SPMC returns the
  number of matching partitions without transferring any descriptors, and the
  caller receives that count in ``*count``.
* When ``out`` is non-NULL, pass the capacity of the caller-supplied
  :c:struct:`ffa_partition_info` array in ``*count``; on return ``*count``
  holds the number of entries written.

Each :c:struct:`ffa_partition_info` descriptor carries the partition ID,
its execution-context count, a property bitmask (:c:macro:`FFA_PARTITION_DIRECT_RECV`
and related ``FFA_PARTITION_*`` macros), and the partition UUID.

**Implementation note — RX-buffer vs register variant:**
When the negotiated FF-A version is 1.2 or later and
``FFA_PARTITION_INFO_GET_REGS`` is available, the register-based variant is
used: partition descriptors are returned in registers (a3 and above) with no
shared-memory transfer and no RX-buffer lock required.  On earlier firmware
(FF-A 1.1 or 1.0) the RX-buffer variant is used instead: the SPMC writes
descriptors into the shared RX buffer, the driver copies them under the
RX-buffer mutex, and then releases the buffer with ``FFA_RX_RELEASE``.  The
caller sees the same :c:struct:`ffa_partition_info` layout regardless of
which path was taken.

Direct Messaging
****************

Two direct-request primitives are provided.

:c:func:`ffa_msg_send_direct_req` sends an ``FFA_MSG_SEND_DIRECT_REQ``
call to a destination endpoint and blocks until the corresponding
``FFA_MSG_SEND_DIRECT_RESP`` is received.  The payload consists of five
``unsigned long`` words (registers x3–x7) carried in
:c:struct:`ffa_send_direct_data`.  On return the same structure holds the
response values from the SP.  Intermediate ``FFA_INTERRUPT`` and
``FFA_YIELD`` indications are handled transparently by re-invoking
``FFA_RUN`` until a final response or error is received.

:c:func:`ffa_msg_send_direct_req2` is the FF-A 1.2 extended variant
(``FFA_MSG_SEND_DIRECT_REQ2``).  It adds a target-service UUID and extends
the payload to fourteen ``unsigned long`` words (registers x4–x17) in
:c:struct:`ffa_send_direct_data2`.  The function returns ``-ENOTSUP`` when
the negotiated FF-A version is below 1.2.

Memory Sharing
**************

Enable with :kconfig:option:`CONFIG_ARM_FFA_MEM_SHARE`.

:c:func:`ffa_mem_share` shares one or more physical memory regions with a
Secure Partition by building an FF-A composite memory-region descriptor in the
TX buffer and issuing ``FFA_FN64_MEM_SHARE``.  On success the SPMC-assigned
64-bit global handle is written into :c:member:`ffa_mem_ops_args.g_handle`.

The caller supplies the borrower's endpoint ID in
:c:member:`ffa_mem_ops_args.dst_id` and an array of
:c:struct:`ffa_mem_region_addr_range` entries describing the physical pages to
share.  Memory is shared as **Normal, Write-Back cacheable, Inner-Shareable,
Read/Write** — the appropriate attributes for NS-to-SP shared memory.

When the descriptor exceeds the TX buffer size, :c:func:`ffa_mem_share`
automatically drives the ``FFA_MEM_FRAG_TX`` loop, sending the remaining
:c:struct:`ffa_mem_region_addr_range` constituents in subsequent fragments until
the SPMC accepts them all and returns ``FFA_SUCCESS_64`` with the final handle.

The descriptor layout is version-aware:

* **FF-A 1.0/1.1** — the per-receiver endpoint memory access descriptor
  (EMAD) is 16 bytes.  ``composite_off`` is 64 (48-byte header + 16-byte EMAD).
* **FF-A 1.2** — EMAD grows to 32 bytes (adds ``impdef_val[16]``).
  ``composite_off`` is 80.

:c:func:`ffa_mem_reclaim` releases a previously shared region.  Pass
``FFA_MEM_RECLAIM_CLEAR`` in ``flags`` to ask the SPMC to zero the memory
before returning it.

Scope
*****

This subsystem provides the foundation for the following planned extensions:

* **SP-4** — notification support.

Configuration
*************

:kconfig:option:`CONFIG_ARM_FFA`
   Enable the FF-A core subsystem. Depends on ``ARM64``. Selects
   :kconfig:option:`CONFIG_ARM_SMCCC_1_2`.

:kconfig:option:`CONFIG_ARM_FFA_MEM_SHARE`
   Enable FF-A memory sharing (``FFA_MEM_SHARE`` / ``FFA_MEM_RECLAIM``).

API Reference
*************

.. doxygengroup:: ffa_core_api
   :project: Zephyr

The public API is declared in ``<zephyr/firmware/ffa.h>``.
