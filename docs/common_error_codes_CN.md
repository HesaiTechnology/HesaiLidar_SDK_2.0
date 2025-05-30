# 常见错误提示
以下是一些您可能会遇到的常见警告代码。

##  故障排查
1. `Failed to obtain correction file from lidar!`  
    这表明该程序试图直接从激光雷达中获取角度校正数据，但未能成功。此后，它仍会尝试从配置参数 `param.input_param.correction_file_path` 指定的本地路径加载校正文件。如果找到了有效的本地角度文件，程序仍可继续运行。

2. `Packet with invaild delimiter`
    这表明该程序要么读取到了格式错误的数据包，要么未能读取到预期的数据包。请检查数据路径并验证数据的完整性。

3. `Firetimes laserId is invalid`
    这表明该程序读取的发光时刻修正文件有误。

4. `Open firetime file failed`
    这表明该程序未能读取发光时刻修正文件，但这并不会阻止程序的运行。