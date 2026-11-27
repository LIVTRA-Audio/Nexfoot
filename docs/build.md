# Build environment

Use `python tools/nexfoot.py build` to create a community firmware package.
The build wrapper compiles the application and runs the JieLi packer offline.

## Dependencies

- Python 3.10+, Git, and the Python packages in `requirements.txt`.
- Official JieLi q32s toolchain. Obtain it from
  https://doc.zh-jieli.com/Tools/zh-cn/dev_tools/dev_env/index.html.
- Windows: toolchain root normally `C:/JL/pi32`, with `bin`, `q32s-lib`, and
  `q32s-include`. Windows uses the pinned SDK's GNU make and vendor packers.
- Linux: toolchain root normally `/opt/jieli/q32s`, with `bin`, `lib`, `include`;
  GNU make and Wine are required. Raise the open-file limit if the linker asks.
- macOS: use Wine for packaging and a compatible q32s compiler for compilation.
  You can also compile in a Linux VM or container with the vendor toolchain.

On macOS/Linux, the wrapper can use a local `linux/amd64` Docker image with the
vendor Linux q32s toolchain installed at `/opt/jieli/q32s`:

```sh
python tools/nexfoot.py build --container YOUR_LOCAL_JL_IMAGE
```

Prepare the image from your own vendor toolchain installation. The wrapper
records the image ID and compiler hash. Wine performs offline packaging
on the host.

`bootstrap` fetches the public SDK commit and verifies the binary dependency
hashes. Keep the SDK cache clean and make source edits in this repository's
`apps`, `cpu`, and `include_lib` directories.

The build uses a fresh stage outside the repository. By default, a checkout
named `Nexfoot` uses the sibling directory `.Nexfoot-work`. Set
`NEXFOOT_WORKSPACE` to choose another location or run
`python tools/nexfoot.py paths` to see the resolved paths. The packager checks
the recovery reference and the size of the application image.

```sh
python tools/nexfoot.py build --toolchain C:/JL/pi32 --version my-build-1
python tools/nexfoot.py inspect dist/nexfoot-my-build-1.nexfoot
```

Packages record source, compiler, ELF, SDK, firmware, and recovery hashes.
Install a compatible q32s toolchain from JieLi. Package contents can vary with
the compiler version.

`.nexfoot` is a ZIP containing only `manifest.json` and `firmware.bin`.
Checksums verify package integrity. Use a firmware package from a source you trust.
