# xnu_kernel_mediatek_mt6781

The MediaTek MT6781 (Helio G96) layer of the kernel, laid over `kernel/mediatek/common` when building: CPU DVFS and
thermal (`osfmk/arm64/mt6781_cpufreq.c`), the GPU probe (`osfmk/arm64/mtk_gpu_probe.c`), MDP, scaler and JPEG.
HakiOS's earlier MT6781 drivers (`mtk/mt6781/`) are in this repository's history up to 4d59e97.

Checked out at `kernel/mediatek/mt6781` by the [Open Mapple manifest](https://github.com/open-mapple/manifest), which says how to get, build and install it.

Licence: viva's files MIT (`LICENSE`); `LICENSE.APSL` for files built into Apple's XNU. Details in `NOTICE`.
