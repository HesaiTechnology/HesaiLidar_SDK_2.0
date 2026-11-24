# Add PCAP replay input manifests and capture integrity statistics

Comparing replay results requires identifying the capture, calibration, configuration, SDK revision and timing settings used. A small machine-readable manifest would support repeatable experiments and useful bug reports.

## Run
```bash
python3 tools/pcap_replay_manifest.py examples/replay_manifest/sample.pcap --correction examples/replay_manifest/correction.csv --config examples/replay_manifest/config.ini --sdk-revision synthetic-example --timestamp-source sensor
```

## Behavior
- Hash PCAP, correction, configuration and optional firetime files with streaming SHA-256.
- Record caller-declared SDK revision, timestamp source and playback rate.
- Inventory classic PCAP records in both byte orders and microsecond/nanosecond precision, including capture-time reversals.
- Detect truncated headers/payloads and invalid record lengths; compare input identities independently of filenames.

## Input and result contract
This utility inventories classic PCAP files and records declared replay settings. Capture record counts include all captured traffic; decoded lidar-frame statistics can be added through an SDK callback in a follow-up. The synthetic capture contains a generic four-byte payload.
The SDK revision and timing settings are supplied by the caller, alongside the configuration file digest. The utility does not execute or parse that configuration. Capture times are reported as integer ticks with ticks_per_second to retain precision. Both classic PCAP byte orders and precisions are supported; PCAPNG needs a separate reader. Invalid/truncated files return exit 2. A valid inventory returns exit 0, with reversals recorded as statistics. comparable(left,right) checks schema, declared settings and all input hashes. No local directory names appear in manifests.

## Tests
10 Python unit tests. See the tests directory.

---

# 新增 PCAP 回放输入清单与捕获完整性统计

比较回放结果需要明确捕获文件、标定、配置、SDK 版本和时间参数。增加机器可读清单，可帮助复现实验并提供完整问题报告。

## 运行
```bash
python3 tools/pcap_replay_manifest.py examples/replay_manifest/sample.pcap --correction examples/replay_manifest/correction.csv --config examples/replay_manifest/config.ini --sdk-revision synthetic-example --timestamp-source sensor
```

## 功能
- 流式计算 PCAP、标定、配置及可选 firetime 文件的 SHA-256。
- 记录调用方声明的 SDK 版本、时间戳来源和回放速率。
- 支持两种字节序、微秒／纳秒精度的 classic PCAP 统计，报告捕获时间倒退。
- 检查头部／数据截断和异常长度，并支持按文件内容而非文件名比较输入。

## 输入与结果
工具清点 classic PCAP 并记录所声明的回放设置。捕获记录数包含全部捕获流量；解码后的雷达帧统计可在后续通过 SDK 回调接入。合成样例使用通用四字节载荷。
SDK 版本和时间参数由调用方声明，同时保留配置文件摘要；工具不执行或解析配置。捕获时间采用整数 tick 和 ticks_per_second，保留原始精度。支持两种字节序、两种精度的 classic PCAP；PCAPNG 需单独读取器。无效／截断文件退出 2；有效清点退出 0，时间倒退作为统计报告。comparable(left,right) 比较 schema、声明参数和全部输入摘要。清单只记录文件名，不含本机目录。

## 测试
10 项 Python 单元测试，详见 tests 目录。
