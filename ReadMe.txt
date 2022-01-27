1: cd TB-8304F_OSC/TB-8304F_OSC
2: export PATH=$PATH:$PWD/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin
3: cd kernel-3.18/
4: make ARCH=arm64 O=out hq8163_tb_a8_n_defconfig
5: make ARCH=arm64 O=out -j32