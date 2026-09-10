1. 中文说明

Python版本 3.7 3.8 3.9
numpy版本1.21.0及以上，opencv-python版本4.2.0.32及以上

windows: win10-x64

目录说明：
	lib            库文件
	Samples        Python测试例子

(1) 配置环境
	请先安装python3 opencv numpy等环境:
	pip3 install opencv-python
	pip3 install numpy

(2)执行Python测试例子：
	2.1 拷贝lib/python_lib/*.pyd和/lib/c_lib/*.dll到Samples目录
	2.2 在Samples目录执行python HelloOrbbec.py等测试例子
	2.3 Samples目录SyncAlignViewer.py测试例子，按D键开关对齐

关于配置文件：
	如果需要修改配置文件，操作如下
	Samples/OrbbecSDKConfig_v1.0.xml，可以按照格式修改配置文件内容
	执行测试程序，程序会读取配置文件


2. English description 

Python version 3.7 3.8 3.9
numpy version 1.21.0 and above
opencv-python version : 4.2.0.32 and above

windows: win10-x64

Directory description:

lib: library file
Samples: Python test example

(1) Configuration environment
Please install python3 opencv numpy and other environments first:
pip3 install opencv-python
pip3 install numpy

(2) Execute the Python test example:
2.1 Copy lib/python_lib/*.pyd and /lib/c_lib/*.dll to the Samples directory
2.2 Execute test examples such as python HelloOrbbec.py in the Samples directory
2.3 SyncAlignViewer.py test example in the Samples directory, press the D key to switch the alignment(depth to color)

About configuration files:
If you need to modify configuration file, the operation is as follows
Samples/OrbbecSDKConfig_v1.0.xml, you can modify the content of the configuration file according to the format
Execute the test program, the program will read the configuration file