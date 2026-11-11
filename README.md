# xnu_kernel_mediatek_mt6781

The MediaTek MT6781 (Helio G96) layer of the kernel, laid over `kernel/mediatek/common` when building:
viva's MT6781-only drivers (CPU DVFS and thermal, MDP, scaler, JPEG), and in `mtk/mt6781/` the realme 8i's
drivers from HakiOS (UFS, touch, keys, LVTS, watchdog), which are being ported to this kernel.

Checked out at `kernel/mediatek/mt6781` by the [Open Mapple manifest](https://github.com/open-mapple/manifest), which says how to get, build and install it.

Licence: `mtk/mt6781/` Apple Public Source License 2.0 (`LICENSE.APSL`); viva's files MIT (`LICENSE`). Details in `NOTICE`.
