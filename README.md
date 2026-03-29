# 毫米波雷达学习仓库

由于我此前从未接触过毫米波雷达，还在学习中，发现相关开源资料过于零散，而且涉及的代码大多已经过时，众多论文难以复现，因此创建此仓库，用于从零开始深入学习毫米波雷达数据处理以及相关算法。

如果您在此仓库中发现任何错误或者有相关建议，欢迎在 `issues` 中提出

更多细节查看 `tutorial` 目录

# 实验条件

数据采集： TI IWR6843ISK + DCA1000EVM (ADC 数据格式为 LVDS Lane 2)
> TI xWR18xx, xWR16xx, xWR68xx 系列都遵循同样的格式，因此都可以使用. 参见 [Mmwave Radar Device ADC Raw Data Capture](https://www.ti.com.cn/lit/an/swra581b/swra581b.pdf?ts=1774154882296&ref_url=https%253A%252F%252Fcn.bing.com%252F)

运行环境：
 - [clang](https://clang.llvm.org/): 需要支持 C23 标准的编译器，不要用 MSVC 
 - [fftw](https://www.fftw.org/download.html)：高效 FFT 计算库
 - [make](https://www.gnu.org/software/make/)

# 快速开始

1. 请确保您已经安装了 支持 C23 标准的 C 编译器， 以及正确配置了 `fftw`

2. 在 Makefile 中修改变量 `CC` 、`FFTW_INCLUDE` 和 `FFTW_LIBRARY`为正确的编译器和 `fftw` 路径

3. 在项目根目录执行

```shell
make run
```
看到类似下面的输出, 则说明环境配置正确

```text
Estimated frames: 5000
Successfully read 5000 frames.

--- First Frame Data (First 5 Samples of Chirp 0, Rx 0) ---
Sample[0]: -213.0000 + -31.0000i
Sample[1]: -23.0000 + -31.0000i
Sample[2]: -167.0000 + -116.0000i
Sample[3]: 14.0000 + -4.0000i
Sample[4]: -121.0000 + -156.0000i
-------------------------------------------------------------
```

# 项目布局

[cmini](https://github.com/storious/cmini): 这来自于我的另一个自娱自乐的 C 工具库

gtrack: (待实现) TI Gtrack 目标跟踪算法

tutorial: 学习记录文档

# 参考资料
- [OpenRadar](https://github.com/PreSenseRadar/OpenRadar.git)
- [TI doc](https://dev.ti.com/tirex/explore/node?isTheia=false&node=A__Af9Z649Xr4Fm41j.zJ.nZg__radar_toolbox__1AslXXD__LATEST)
