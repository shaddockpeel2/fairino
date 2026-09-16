# RKNNLite runtime wheel

`rknn_toolkit_lite2-2.3.2-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl`
is copied from the board's existing `/home/PPOCR/images/` runtime assets.
SHA-256: `821e80c95e6838308c133915660b1a6ae78bb8d079b2cbbd46a02dae61192d33`.

Run `scripts/setup_yolo_runtime.sh` once. It installs this wheel into the
workspace-local `.venv-yolo` without downloading packages or altering the
system Python. The launcher adds that environment to `PYTHONPATH` only for
`detection_mode:=yolo`. NumPy and psutil come from the board's ROS/system
Python environment.

`ruamel_runtime_py310_aarch64.tar.gz` is a copy of the old project's installed
`ruamel.yaml==0.18.15` and `ruamel.yaml.clib==0.2.15` runtime files, including
their distribution metadata and license files. SHA-256:
`08132e52d754392d97dc3c8ce5beb54c9256c7ba4df1f9ea2a1033fd8de24da6`.
