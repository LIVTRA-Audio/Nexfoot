# SPDX-License-Identifier: Apache-2.0
"""Apply the Nexfoot WebHID boot overlay to a clean pinned JieLi source tree."""
from pathlib import Path
import shutil
import subprocess
import sys

OVERLAY = Path(__file__).resolve().parent
VENDOR = Path(sys.argv[1]).resolve()
EXPECTED = "ff3b9299d1b94d834b995fd858b3f75e2faa9840"

def replace(path, before, after):
    data = path.read_bytes().replace(b"\r\n", b"\n")
    old = before.encode()
    if data.count(old) != 1:
        raise ValueError(f"Unexpected JieLi source at {path}: {before}")
    path.write_bytes(data.replace(old, after.encode()))

head = subprocess.check_output(["git", "-C", VENDOR, "rev-parse", "HEAD"], text=True).strip()
if head != EXPECTED:
    raise ValueError("Unexpected JieLi boot revision")
subprocess.run(["git", "-C", VENDOR, "apply", OVERLAY.parent / "recovery_main.patch"], check=True)
sources = VENDOR / "user_boot/app/src"
shutil.copy2(OVERLAY / "web_update.c", sources / "user.c")

# Use a different VID:PID from both runtime and mask UBOOT. This PID is for
# this bench prototype. Confirm its allocation before publishing firmware.
descriptor = sources / "descriptor.c"
replace(descriptor, "'U', 'A',", "0x57, 0x41,")
replace(descriptor, "0x02,       // iProduct:", "0x00,       // iProduct:")

main = sources / "main.c"
replace(main, "static u8 md400_recovery_key_held(void)", "static u8 md400_update_key(void)")
replace(main, "            return 0;\n        }\n        held_ms += MD400_RECOVERY_POLL_MS;",
        "            return held_ms >= 600 && md400_usb_cable_present() ? 2 : 0;\n        }\n        held_ms += MD400_RECOVERY_POLL_MS;")
replace(main, '    if (md400_recovery_key_held()) {\n        md400_enter_mask_uboot("PB1 long hold");\n    }',
        '''    extern void nexfoot_web_update(void);
    extern u32 nvram_list[];
    u8 update_mode = md400_update_key();
    u32 web_request = nvram_list[0];
    if (web_request == 0x4857464eu) {
        nvram_list[0] = 0;
    }
    if (update_mode == 1) {
        md400_enter_mask_uboot("PB1 long hold");
    }
    if (md400_usb_cable_present() && (update_mode == 2 || web_request == 0x4857464eu)) {
        nexfoot_web_update();
    }''')

# Interrupt IN/OUT reports comprise the updater protocol. Retain the upstream
# enumeration and report descriptor handlers but drop its unused demo
# control-report receiver and functional descriptor strings.
hid = sources / "hid.c"
hid_source = hid.read_text(encoding="utf-8")
start = hid_source.index("static u32 hid_recv_output_report(")
end = hid_source.index("static u32 hid_itf_hander(", start)
hid_source = hid_source[:start] + hid_source[end:]
hid_source = hid_source.replace(
    "            usb_set_setup_recv(usb_device, hid_recv_output_report);",
    "            usb_set_setup_phase(usb_device, USB_EP0_SET_STALL);",
)
hid.write_text(hid_source, encoding="utf-8", newline="\n")
replace(descriptor, "0x01,       // iManufacturer:", "0x00,       // iManufacturer:")
descriptor_source = descriptor.read_text(encoding="utf-8")
for name in ("get_manufacture_str", "get_product_str"):
    start = descriptor_source.index("    memcpy(", descriptor_source.index("void " + name))
    end = descriptor_source.index("\n", start)
    descriptor_source = descriptor_source[:start] + "    (void)ptr;" + descriptor_source[end:]
descriptor.write_text(descriptor_source, encoding="utf-8", newline="\n")

