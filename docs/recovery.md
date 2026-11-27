# Recovery and updates

With one Nexfoot connected by USB, hold its power button during startup for
two seconds to enter UBOOT. This recovery mode can start independently of
the application firmware.

Run the [community updater](../README.md#update-and-recovery) in preflight mode
first. It checks the device and the installed recovery boot against the package.
Older units may need a device-specific migration before they can use community
packages. If preflight rejects the device, stop and check compatibility.

With `--yes`, the updater saves and verifies a full backup before writing
application sectors. It preserves recovery and reserved storage, including
presets on compatible layouts. It verifies the image before starting the
application and checks that USB MIDI returns.

If an update is interrupted, re-enter UBOOT and retry the same compatible
package. Keep backups in the local workspace's `backups` directory. They
contain presets and device identity.

The Windows updater uses SCSI pass-through. On Linux and macOS, install libusb
and ensure the host can access the UBOOT interface. macOS may require administrator
access. The community updater still requires hardware qualification for general use.

## What recovery covers

Recovery lives in the first 8 KiB of flash, the protected recovery boot (see
[`recovery/`](../recovery/README.md)). It does not depend on the application
firmware. It enters the chip's built-in BD19 UBOOT USB updater in two cases:

- USB is connected and the power button is held for two seconds at startup.
- The application cannot start because its file system is damaged or the jump
  to the application fails. The pedal then enters UBOOT on its own, without the
  button. A pedal that shows nothing after a failed update may already be
  waiting in UBOOT, so run the updater preflight.

Normal updates never write the recovery boot. The community updater, Creator
Lab and LIVTRA's signed `.fmu` packages all keep it and check it against its
pinned SHA-256.

Recovery stays available only while those 8 KiB are intact. They can still be
overwritten by:

- JieLi's ISD download tool, or a modified updater that writes the whole flash.
- Application firmware. The firmware has flash write access and nothing in it
  blocks writes to the recovery region. Do not write flash below the
  application area in your own builds.

A Nexfoot whose recovery boot is erased or damaged can no longer enter UBOOT
from the button. Restoring it needs a full flash through JieLi's download tool
and hardware access to the board. Contact LIVTRA support before trying this.
