# 编译宏控制，生效或失效
以下是您需要使得某个编译宏生效或失效时，需要的操作。

下述以宏 WITH_PTCS_USE 为例

##  Ubuntu下说明
- 方式1： `cmake .. -DWITH_PTCS_USE=true` 或 `cmake .. -DWITH_PTCS_USE=false`

    1. 在build文件夹下使用上述命令

    2. 当为true时代表生效，为false时代表失效

    3. 如果出现报错，请先使用`rm ./* -r`清空build文件夹后重试

- 方式2： 使用cmake-gui工具

    1. `sudo apt install cmake-gui` 下载安装
    
    2. `cmake-gui` 运行

    3. `Where is the source code` 选择本地 `HesaiLidar_SDK_2.0` 目录， `Where to build the binaries` 选择本地 `HesaiLidar_SDK_2.0/build` 目录

    4. 在界面中找到 `WITH_PTCS_USE`选择，将其Value值勾选为生效，去掉勾选为失效。 一般该选择在 `WITH` 栏中。

    5. 如果无法找到该选项，可以在 `Add Entry` 中添加，`Type` 选择为 `BOOL` 类型即可。

    6. 点击 `Configure` 和 `Generate` 按钮配置工程

## Window下说明
    
参考 **[如何在Windows下编译SDK](docs/compile_on_windows_CN.md)** 中的`2 CMake配置`，使用 `CMake GUI` 工具配置。

在界面中找到 `WITH_PTCS_USE`选择，将其Value值勾选为生效，去掉勾选为失效。

如果无法找到该选项，可以在 `Add Entry` 中添加，`Type` 选择为 `BOOL` 类型即可。