# obsensor_metadata_win10

由于 Windows 系统限制，默认情况下无法通过 UVC 协议获取到设备时间戳，需要修改注册表完成注册后才能获取。

1. 连接设备，确认设备已上线；
2. 通过管理员权限打开 powershell ，然后 `cd` 命令进入到 `scripts` 目录；
3. 执行 `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser` 命令，按提示输入 `Y` 确认;
4. 执行 `.\obsensor_metadata_win10.ps1 -op install_all` 完成注册。
