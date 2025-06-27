# 常见错误提示
以下是一些您可能会遇到的常见警告代码。

##  故障排查
1. `Failed to obtain correction file from lidar!` 

    这表明该程序试图直接从激光雷达中获取角度校正数据，但未能成功。此后，它仍会尝试从配置参数 `param.input_param.correction_file_path` 指定的本地路径加载校正文件。如果找到了有效的本地角度文件，程序仍可继续运行。

2. `Packet with invaild delimiter`

    这表明该程序读到了非预期的数据包，如果偶尔打印，则不影响解析，仅代表接收到一些无法解析的包数据。如果一直打印，则代表一直收到非预期的包数据，可能是该雷达的数据暂不支持解析，或雷达只发送fault message报文，未发送点云报文，或其他原因。

3. `Firetimes laserId is invalid`
    这表明该程序读取的发光时刻修正文件有误。

4. `Open firetime file failed`
    这表明该程序未能读取发光时刻修正文件，但这并不会阻止程序的运行。