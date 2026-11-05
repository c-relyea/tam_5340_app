# Source this before building:  . tools/ncs_env.sh
#
# Puts the NCS v2.8.0 toolchain ahead of Homebrew on PATH for this shell only.
# ~/.zshrc prepends $(brew --prefix)/bin after the toolchain, so a plain shell
# picks up Homebrew's CMake 4.x, which NCS v2.8.0 can't configure with.
# Mirrors the toolchain's environment.json.

NCS_TOOLCHAIN_PATH=/opt/nordic/ncs/toolchains/15b490767d
export NCS_TOOLCHAIN_PATH

PATH="$NCS_TOOLCHAIN_PATH/bin:$NCS_TOOLCHAIN_PATH/usr/bin:$NCS_TOOLCHAIN_PATH/usr/local/bin:$NCS_TOOLCHAIN_PATH/opt/bin:$NCS_TOOLCHAIN_PATH/opt/nanopb/generator-bin:$NCS_TOOLCHAIN_PATH/opt/zephyr-sdk/arm-zephyr-eabi/bin:$NCS_TOOLCHAIN_PATH/opt/zephyr-sdk/riscv64-zephyr-elf/bin:$PATH"
export PATH

export GIT_EXEC_PATH="$NCS_TOOLCHAIN_PATH/Cellar/git/2.37.3/libexec/git-core"
export GIT_TEMPLATE_DIR="$NCS_TOOLCHAIN_PATH/Cellar/git/2.37.3/share/git-core/templates"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR="$NCS_TOOLCHAIN_PATH/opt/zephyr-sdk"
