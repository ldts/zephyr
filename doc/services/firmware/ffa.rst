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

Scope
*****

This subsystem provides the foundation for the following planned extensions:

* **SP-2b** — partition-info discovery and direct messaging.
* **SP-3** — memory-sharing operations.
* **SP-4** — notification support.

None of SP-2b, SP-3, or SP-4 are included in this phase.

Configuration
*************

:kconfig:option:`CONFIG_ARM_FFA`
   Enable the FF-A core subsystem. Depends on ``ARM64``. Selects
   :kconfig:option:`CONFIG_ARM_SMCCC_1_2`.

API Reference
*************

.. doxygengroup:: ffa_core_api
   :project: Zephyr

The public API is declared in ``<zephyr/firmware/ffa.h>``.
