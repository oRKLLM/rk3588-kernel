# Board kernel snapshots

The RK3588 board (`10.3.0.236`) builds its kernel from **its own working tree**
(`~/kbuild/linux-rockchip`), which `ork-driver/tools/util/vm_kbuild.sh` syncs into the build VM. That tree
is the *diagnostic* tree and is not any branch here: it carries ~49 numbered hunks, most of which are
deliberately not upstreamable and appear in neither `develop-6.1` nor the fix branches.

The consequence is a trap. Any edit left in that tree by anyone is silently included in the next kernel
**anybody** builds, and gets attributed to whatever change that person thought they were testing. A kernel
version alone (`uname -v`) therefore does not identify what was running.

These snapshot branches exist so a board kernel is reproducible from git after the fact. Each captures
exactly what `vm_kbuild.sh` syncs — `drivers/rknpu/**`, `drivers/iommu/rockchip-iommu.c`, and the
`.config` (stored here as `rk3588-board-<N>.config`, since `.config` is gitignored) — on top of the vendor
base, so `git diff develop-6.1..board-snapshot-<N>` is the full delta that was actually running.

To build a snapshot instead of the live board tree:

    ork-driver/tools/util/vm_kbuild.sh --from-branch board-snapshot-59

See the wiki, *Kernel-Modifications → What the board is actually running*, for the per-hunk inventory.
